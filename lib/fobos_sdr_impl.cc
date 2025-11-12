//==============================================================================
//       _____     __           _______
//      /  __  \  /_/          /  ____/                                __
//     /  /_ / / _   ____     / /__  __  __   ____    ____    ____   _/ /_
//    /    __ / / / /  _  \  / ___/  \ \/ /  / __ \  / __ \  / ___\ /  _/
//   /  /\ \   / / /  /_/ / / /___   /   /  / /_/ / /  ___/ / /     / /_
//  /_ /  \_\ /_/  \__   / /______/ /_/\_\ / ____/  \____/ /_/      \___/
//               /______/                 /_/             
//  Fobos SDR API library
//  V.V.
//  2024.03.21
//  2024.04.08
//  2024.04.21
//  2024.04.26
//  2024.08.10 libfobos 2.3.1 support, freq[Hz]
//==============================================================================
#include <math.h>
#include "fobos_sdr_impl.h"
#include <gnuradio/io_signature.h>

namespace gr
{
    namespace RigExpert
    {
        //======================================================================
        using output_type = gr_complex;
        fobos_sdr::sptr fobos_sdr::make(int index,
                                        double warmup_frequency,
                                        const std::vector<double> &frequencies, 
                                        double samplerate,
                                        int lna_gain,
                                        int vga_gain,
                                        int direct_sampling,
                                        int clock_source,
                                        int buf_len,
                                        int exclude_before_phase,
                                        int exclude_after_phase)
        {
            return gnuradio::make_block_sptr<fobos_sdr_impl>(
                                        index,
                                        warmup_frequency,
                                        frequencies,
                                        samplerate,
                                        lna_gain,
                                        vga_gain,
                                        direct_sampling,
                                        clock_source,
                                        buf_len,
                                        exclude_before_phase,
                                        exclude_after_phase);
        }
        //======================================================================
        // The private constructor
        fobos_sdr_impl::fobos_sdr_impl( int index,
                                        double warmup_frequency,
                                        const std::vector<double> &frequencies,
                                        double samplerate,
                                        int lna_gain,
                                        int vga_gain,
                                        int direct_sampling,
                                        int clock_source,
                                        int buf_len,
                                        int exclude_before_phase,
                                        int exclude_after_phase)
            : gr::sync_block("fobos_sdr",
                             gr::io_signature::make(0, 0, 0),
                             gr::io_signature::make(
                                 1, 1, sizeof(output_type)))
        {
                char lib_version[64];
                char drv_version[64];
                char hw_revision[64];
                char fw_version[64];
                char manufacturer[64];
                char product[64];
                char serial[64];

            _frequencies = frequencies;
            _rx_bufs = 0;
            _rx_idx_w = 0;
            _rx_pos_r = 0;
            _rx_idx_r = 0;
            _rx_pos_w = 0;
            _rx_filled = 0;
            _running = false;
            _frequency = -1;
            _lna_gain = lna_gain;
            _vga_gain = vga_gain;
            _sample_rate = samplerate;
            _warmup = true;
            _is_scanning = false;
            _phase = 0;
            _sample_count = 0;

            fobos_sdr_get_api_info(lib_version, drv_version);
            printf("Fobos SDR API Info lib: %s drv: %s\n", lib_version, drv_version);
        
            int count = fobos_sdr_get_device_count();

            printf("fobos_sdr_impl:: found devices: %d\n", count);
            if (count == 0)
            {
                printf("could not find any fobos_sdr compatible device!\n");
                return;
            }
            int result = 0;

            result = fobos_sdr_open(&_dev, index);

            if (result != 0)
            {
                printf("could not open device! err (%i)\n", result);
                return;
            }

            printf("open...ok\n");
            result = fobos_sdr_get_board_info(_dev, hw_revision, fw_version, manufacturer, product, serial);
            if (result != 0)
            {
                printf("fobos_sdr_get_board_info - error!\n");
            }
            else
            {
                printf("board info\n");
                printf("    hw_revision:  %s\n", hw_revision);
                printf("    fw_version:   %s\n", fw_version);
                printf("    manufacturer: %s\n", manufacturer);
                printf("    product:      %s\n", product);
                printf("    serial:       %s\n", serial);
            }

            set_frequency(warmup_frequency);
            set_samplerate(samplerate);
            set_lna_gain(0);
            set_vga_gain(0);
            set_direct_sampling(direct_sampling);
            set_clock_source(clock_source);            

            _rx_buffs_count = 32;
            _rx_buff_len = 8192 * buf_len;

            _rx_bufs = (float**)malloc(_rx_buffs_count * sizeof(float*));
            for (unsigned int i = 0; i < _rx_buffs_count; i++)
            {
                _rx_bufs[i] = (float*)malloc(_rx_buff_len * 2 * sizeof(float));
            }

            _exclude_half_width = (exclude_after_phase - exclude_before_phase) / 2;
            _exclude_offset = exclude_before_phase + _exclude_half_width;

            _running = true;
            _thread = gr::thread::thread(thread_proc, this);
            _thread_started = true;

            _vec_running = true;
            _vec_thread = gr::thread::thread(vector_loop, this);
            _vec_thread_started = true;

            message_port_register_in(pmt::mp("phase"));
            set_msg_handler(pmt::mp("phase"), [this](pmt::pmt_t msg) {
                this->handle_phase_msg(msg);
            });
            message_port_register_in(pmt::mp("control"));
            set_msg_handler(pmt::mp("control"), [this](pmt::pmt_t msg) {
                this->handle_control_msg(msg);
            });
            message_port_register_out(pmt::mp("vector"));
        }
        //======================================================================
        // virtual destructor
        fobos_sdr_impl::~fobos_sdr_impl()
        {
            if (_dev)
            {
                fobos_sdr_cancel_async(_dev);
            }
            if (_thread_started)
            {
                _thread.join();
            }
            _vec_running = false;
            if (_vec_thread_started)
            {
                _vec_thread.join();
            }
            if (_dev)
            {
                stop_scan();
                fobos_sdr_close(_dev);
            }
            if (_rx_bufs)
            {
                for (unsigned int i = 0; i < _rx_buffs_count; i++)
                {
                    if (_rx_bufs[i])
                    {
                        free(_rx_bufs[i]);
                    }
                }
                free(_rx_bufs);
            }
        }
        //======================================================================
        // Handle pmt input
        void fobos_sdr_impl::handle_phase_msg(pmt::pmt_t msg)
        {
            if (pmt::is_integer(msg))
            {
                _phase = pmt::to_uint64(msg);
            }
            _warmup = false;
            set_lna_gain(_lna_gain);
            set_vga_gain(_vga_gain);
            set_frequency(_frequency);
        }
        void fobos_sdr_impl::handle_control_msg(pmt::pmt_t msg)
        {
            if (!pmt::is_dict(msg)) return;
            pmt::pmt_t v_freq = pmt::dict_ref(msg, pmt::intern("freq"), pmt::PMT_NIL);
            if (pmt::is_number(v_freq))
            {
                _frequency = pmt::to_double(v_freq);
                if (!_warmup) set_frequency(_frequency);
            }
            pmt::pmt_t v_lna = pmt::dict_ref(msg, pmt::intern("lna"), pmt::PMT_NIL);
            if (pmt::is_number(v_lna))
            {
                _lna_gain = pmt::to_long(v_lna);
                if (!_warmup) set_lna_gain(_lna_gain);
            }
            pmt::pmt_t v_vga = pmt::dict_ref(msg, pmt::intern("vga"), pmt::PMT_NIL);
            if (pmt::is_number(v_vga))
            {
                _vga_gain = pmt::to_double(v_vga);
                if (!_warmup) set_vga_gain(_vga_gain);
            }
        }
        //======================================================================
        // Work
        int fobos_sdr_impl::work(int noutput_items,
                                 gr_vector_const_void_star& input_items,
                                 gr_vector_void_star& output_items)
        {
            auto out = static_cast<output_type*>(output_items[0]);
            std::unique_lock<std::mutex> lock(_rx_mutex);
            _rx_cond.wait(lock, [&]{ return !_running || _rx_filled > 0; });
            if (!_running && _rx_filled == 0) return 0;
            int produced = 0;
            while (this->_rx_filled > 0 && produced < noutput_items)
            {
                float * buff = _rx_bufs[_rx_idx_r] + _rx_pos_r * 2;
                size_t samples_count = _rx_buff_len - _rx_pos_r;
                if (samples_count > static_cast<size_t>(noutput_items - produced))
                {
                    samples_count = noutput_items - produced;
                }
                lock.unlock();
                memcpy(reinterpret_cast<float*>(out) + produced * 2, buff, samples_count * 2 * sizeof(float));
                lock.lock();
                _rx_pos_r += samples_count;
                if (_rx_pos_r >= _rx_buff_len)
                {
                    _rx_pos_r = 0;
                    _rx_idx_r = (_rx_idx_r + 1) % _rx_buffs_count;
                    _rx_filled--;
                }
                produced += static_cast<int>(samples_count);
            }
            return produced;
        }
        //======================================================================
        void fobos_sdr_impl::read_samples_callback(float *buf, uint32_t buf_length, struct fobos_sdr_dev_t* sender, void *user)
        {
            fobos_sdr_impl* _this = static_cast<fobos_sdr_impl*>(user);
            if (_this->_rx_buff_len != buf_length)
            {
                printf("Err: wrong buf_length!!!");
                printf("canceling...");
                fobos_sdr_cancel_async(sender);
                return;
            }

            uint32_t prev_sample_count = _this->_sample_count;
            _this->_sample_count += buf_length;
            if (_this->_sample_count >= _this->_sample_rate) _this->_sample_count -= _this->_sample_rate;

            const bool is_scanning = fobos_sdr_is_scanning(sender);
            const int channel = fobos_sdr_get_scan_index(sender);
            {
                std::lock_guard<std::mutex> lock(_this->_rx_mutex);
                if (_this->_rx_filled < _this->_rx_buffs_count)
                {
                    if (is_scanning && channel == -1)
                    {
                        memset(_this->_rx_bufs[_this->_rx_idx_w], 0, _this->_rx_buff_len * 2 * sizeof(float));
                    }
                    else
                    {
                        memcpy(_this->_rx_bufs[_this->_rx_idx_w], buf, _this->_rx_buff_len * 2 * sizeof(float));
                    }
                    _this->_rx_idx_w = (_this->_rx_idx_w + 1) % _this->_rx_buffs_count;
                    _this->_rx_filled++;
                }
                else
                {
                    printf("OVERRUN!!!");
                }
                _this->_rx_cond.notify_one();
            }
            if (is_scanning && channel != -1)
            {
                uint32_t d = _this->_phase + _this->_exclude_offset + _this->_sample_rate - prev_sample_count;
                if (d >= _this->_sample_rate) d -= _this->_sample_rate;
                uint32_t dist = (d < buf_length) ? 0u : std::min(d - buf_length, _this->_sample_rate - d);
                if (dist > _this->_exclude_half_width)
                {
                    std::lock_guard<std::mutex> lock(_this->_vec_mutex);
                    std::vector<gr_complex> vec(_this->_rx_buff_len);
                    memcpy(reinterpret_cast<float*>(vec.data()), buf, _this->_rx_buff_len * 2 * sizeof(float));
                    _this->_vec_queue.push({_this->_frequencies[channel], std::move(vec)});
                    _this->_vec_cond.notify_one();
                }
            }
        }
        //======================================================================
        void fobos_sdr_impl::thread_proc(fobos_sdr_impl * _this)
        {
            int result = fobos_sdr_read_async(_this->_dev, read_samples_callback, _this, 16, _this->_rx_buff_len);
            if (result == 0)
            {
                printf("fobos_sdr_read_async - ok!\n");
            }
            else
            {
                printf("fobos_sdr_read_async - error!\n");
            }
            {
                std::lock_guard<std::mutex> lock(_this->_rx_mutex);
                _this->_running = false;
            }
            _this->_rx_cond.notify_all();
        }
        //======================================================================
        void fobos_sdr_impl::vector_loop(fobos_sdr_impl * _this)
        {
            while (_this->_vec_running)
            {
                std::unique_lock<std::mutex> lock(_this->_vec_mutex);
                _this->_vec_cond.wait(lock, [&]{ return !_this->_vec_running || !_this->_vec_queue.empty(); });
                if (!_this->_vec_running) break;
                if (_this->_vec_queue.empty()) continue;
                auto [channel, vec] = std::move(_this->_vec_queue.front());
                _this->_vec_queue.pop();
                lock.unlock();
                pmt::pmt_t meta = pmt::make_dict();
                meta = pmt::dict_add(meta, pmt::intern("channel"), pmt::from_double(channel));
                meta = pmt::dict_add(meta, pmt::intern("samples"), pmt::init_c32vector(_this->_rx_buff_len, vec.data()));
                _this->message_port_pub(pmt::mp("vector"), meta);
            }
        }
        //======================================================================
        void fobos_sdr_impl::start_scan()
        {
            if (_is_scanning) return;
            int res = fobos_sdr_start_scan(_dev, _frequencies.data(), _frequencies.size());
            _is_scanning = res == 0;
            printf("Starting scanning: %s\n", res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::stop_scan()
        {
            if (!_is_scanning) return;
            int res = fobos_sdr_stop_scan(_dev);
            _is_scanning = res != 0;
            printf("Stopping scanning: %s\n", res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::set_frequency(double frequency)
        {
            if (frequency == -1)
            {
                start_scan();
                return;
            }
            stop_scan();
            int res = fobos_sdr_set_frequency(_dev, frequency);
            printf("Setting freq %f MHz: %s\n", frequency / 1E6, res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::set_samplerate(double samplerate)
        {
            int res = fobos_sdr_set_samplerate(_dev, samplerate);
            printf("Setting sample rate %f MHz: %s\n", samplerate / 1E6, res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::set_lna_gain(int lna_gain)
        {
            int res = fobos_sdr_set_lna_gain(_dev, lna_gain);
            printf("Setting LNA gain to #%d: %s\n", lna_gain, res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::set_vga_gain(int vga_gain)
        {
            int res = fobos_sdr_set_vga_gain(_dev, vga_gain);
            printf("Setting VGA gain to #%d: %s\n",  vga_gain, res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::set_direct_sampling(int direct_sampling)
        {
            int res = fobos_sdr_set_direct_sampling(_dev, direct_sampling);
            printf("Setting direct sampling mode to %d: %s\n",  direct_sampling, res == 0 ? "OK" : "ERR");
        }
        //======================================================================
        void fobos_sdr_impl::set_clock_source(int clock_source)
        {
            int res = fobos_sdr_set_clk_source(_dev, clock_source);
            printf("Setting clock source to %s: %s\n",  clock_source == 0 ? "internal" : "external", res == 0 ? "OK" : "ERR");
        }
        //======================================================================
    } /* namespace RigExpert */
} /* namespace gr */
