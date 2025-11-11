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

#ifndef INCLUDED_RIGEXPERT_FOBOS_SDR_IMPL_H
#define INCLUDED_RIGEXPERT_FOBOS_SDR_IMPL_H

#include <gnuradio/sync_block.h>
#include <gnuradio/thread/thread.h>
#include <mutex>
#include <condition_variable>
#include <gnuradio/RigExpert/fobos_sdr.h>
#include <fobos_sdr.h>
#include <pmt/pmt.h>
#include <queue>

namespace gr
{
    namespace RigExpert
    {
        class fobos_sdr_impl : public fobos_sdr
        {
        private:
            uint32_t _buff_counter;
            std::queue<std::pair<double, std::vector<gr_complex>>> _vec_queue;
            bool _running;
            bool _vec_running;
            gr::thread::thread _thread;
            gr::thread::thread _vec_thread;
            bool _thread_started = false;
            bool _vec_thread_started = false;
            std::mutex _rx_mutex;
            std::mutex _vec_mutex;
            std::condition_variable _rx_cond;
            std::condition_variable _vec_cond;
            float ** _rx_bufs;
            size_t _rx_buffs_count;
            size_t _rx_buff_len;
            size_t _rx_filled;
            size_t _rx_idx_w;
            size_t _rx_pos_w;
            size_t _rx_idx_r;
            size_t _rx_pos_r;
            std::vector<double> _frequencies;
            double _frequency;
            int _lna_gain;
            int _vga_gain;
            bool _warmup;
            bool _is_scanning;
            struct fobos_sdr_dev_t * _dev = NULL;
            static void read_samples_callback(float * buf, uint32_t buf_length, struct fobos_sdr_dev_t* sender, void * user);
            static void thread_proc(fobos_sdr_impl * ctx);
            static void vector_loop(fobos_sdr_impl * ctx);
            void handle_end_warmup_msg(pmt::pmt_t msg);
            void handle_control_msg(pmt::pmt_t msg);
        public:
            fobos_sdr_impl( int index,
                            double warmup_frequency,
                            const std::vector<double> &frequencies,
                            double samplerate,
                            int lna_gain,
                            int vga_gain,
                            int direct_sampling,
                            int clock_source,
                            int buf_len);
            ~fobos_sdr_impl();

            int work(int noutput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

            void start_scan();
            void stop_scan();
            void set_frequency(double frequency);
            void set_samplerate(double samplerate);
            void set_lna_gain(int lna_gain);
            void set_vga_gain(int vga_gain);
            void set_direct_sampling(int direct_sampling);
            void set_clock_source(int clock_source);
        };

    } // namespace RigExpert
} // namespace gr

#endif /* INCLUDED_RIGEXPERT_FOBOS_SDR_IMPL_H */

//==============================================================================