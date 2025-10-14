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
                                        double frequency,
                                        double samplerate,
                                        int lna_gain,
                                        int vga_gain,
                                        int direct_sampling,
                                        int clock_source)
        {
            printf("make (%d, %f, %f, %f, %d, %d, %d, %d)\n", index, warmup_frequency, frequency, samplerate, lna_gain, vga_gain, direct_sampling, clock_source);
            return gnuradio::make_block_sptr<fobos_sdr_impl>(
                                        index,
                                        warmup_frequency,
                                        frequency, 
                                        samplerate,
                                        lna_gain,
                                        vga_gain,
                                        direct_sampling,
                                        clock_source);
        }
        //======================================================================
        // The private constructor
        fobos_sdr_impl::fobos_sdr_impl( int index,
                                        double warmup_frequency,
                                        double frequency, 
                                        double samplerate,
                                        int lna_gain,
                                        int vga_gain,
                                        int direct_sampling,
                                        int clock_source)
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

            _rx_bufs = 0;
            _rx_idx_w = 0;
            _rx_pos_r = 0;
            _rx_idx_r = 0;
            _rx_pos_w = 0;
            _rx_filled = 0;
            _running = false;
            _buff_counter = 0;
            _frequency = frequency;
            _lna_gain = lna_gain;
            _vga_gain = vga_gain;
            _warmup = true;
            fobos_sdr_get_api_info(lib_version, drv_version);
            printf("Fobos SDR API Info lib: %s drv: %s\n", lib_version, drv_version);
        
            int count = fobos_sdr_get_device_count();

            printf("fobos_sdr_impl:: found devices: %d\n", count);
            if (count > 0)
            {
                int result = 0;

                result = fobos_sdr_open(&_dev, index);

                if (result == 0)
                {
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
                    printf("(%d, %f, %f, %d, %d, %d, %d)\n", index, warmup_frequency, samplerate, 0, 0, direct_sampling, clock_source);

                    result = fobos_sdr_set_frequency(_dev, warmup_frequency);
                    if (result != 0)
                    {
                        printf("fobos_sdr_set_frequency - error!\n");
                    }

                    result = fobos_sdr_set_samplerate(_dev, samplerate);
                    if (result != 0)
                    {
                        printf("fobos_sdr_set_samplerate - error!\n");
                    }

                    result = fobos_sdr_set_lna_gain(_dev, 0);
                    if (result != 0)
                    {
                        printf("fobos_sdr_set_lna_gain - error!\n");
                    }

                    result = fobos_sdr_set_vga_gain(_dev, 0);
                    if (result != 0)
                    {
                        printf("fobos_sdr_set_vga_gain - error!\n");
                    }

                    result = fobos_sdr_set_direct_sampling(_dev, direct_sampling);
                    if (result != 0)
                    {
                        printf("fobos_sdr_set_direct_sampling - error!\n");
                    }
                    
                    result = fobos_sdr_set_clk_source(_dev, clock_source);
                    if (result != 0)
                    {
                        printf("fobos_sdr_set_clk_source - error!\n");
                    }

                    _rx_buffs_count = 32;
                    _rx_buff_len = 65536*2;

                    _rx_bufs = (float**)malloc(_rx_buffs_count * sizeof(float*));
                    for (unsigned int i = 0; i < _rx_buffs_count; i++)
                    {
                        _rx_bufs[i] = (float*)malloc(_rx_buff_len * 2 * sizeof(float));
                    }

                    _running = true;
                    _thread = gr::thread::thread(thread_proc, this);
                    _thread_started = true;
                }
                else
                {
                    printf("could not open device! err (%i)\n", result);
                }
            }
            else
            {
                printf("could not find any fobos_sdr compatible device!\n");
            }
            message_port_register_in(pmt::mp("end_warmup"));
            set_msg_handler(pmt::mp("end_warmup"), [this](pmt::pmt_t msg) {
                this->handle_end_warmup_msg(msg);
            });
            message_port_register_in(pmt::mp("control"));
            set_msg_handler(pmt::mp("control"), [this](pmt::pmt_t msg) {
                this->handle_control_msg(msg);
            });
        }
        //======================================================================
        // virtual destructor
        fobos_sdr_impl::~fobos_sdr_impl()
        {
            if (_dev)
            {
                fobos_sdr_cancel_async(_dev);
            }
            if (_thread_started) {
                _thread.join();
            }
            if (_dev)
            {
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
        void fobos_sdr_impl::handle_end_warmup_msg(pmt::pmt_t msg)
        {
            _warmup = false;
            set_frequency(_frequency);
            set_lna_gain(_lna_gain);
            set_vga_gain(_vga_gain);
        }
        void fobos_sdr_impl::handle_control_msg(pmt::pmt_t msg)
        {
            if (!pmt::is_dict(msg)) return;
            pmt::pmt_t v_freq = pmt::dict_ref(msg, pmt::intern("freq"), pmt::PMT_NIL);
            if (pmt::is_number(v_freq)) _frequency = pmt::to_double(v_freq);
            pmt::pmt_t v_lna = pmt::dict_ref(msg, pmt::intern("lna"), pmt::PMT_NIL);
            if (pmt::is_number(v_lna)) _lna_gain = pmt::to_long(v_lna);
            pmt::pmt_t v_vga = pmt::dict_ref(msg, pmt::intern("vga"), pmt::PMT_NIL);
            if (pmt::is_number(v_vga)) _vga_gain = pmt::to_double(v_vga);
            if (!_warmup) {
                set_frequency(_frequency);
                set_lna_gain(_lna_gain);
                set_vga_gain(_vga_gain);
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
            std::lock_guard<std::mutex> lock(_this->_rx_mutex);
            if (_this->_rx_filled < _this->_rx_buffs_count)
            {
                memcpy(_this->_rx_bufs[_this->_rx_idx_w], buf, _this->_rx_buff_len * 2 *sizeof(float));
                _this->_rx_idx_w = (_this->_rx_idx_w + 1) % _this->_rx_buffs_count;
                _this->_rx_filled++;
            }
            else
            {
                printf("OVERRUN!!!");
            }
            _this->_rx_cond.notify_one();
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
        void fobos_sdr_impl::set_frequency(double frequency)
        {
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
