#pragma once

#include <cmath>
#include <utility>
#include <stdint.h>
#include <arduino.h>

namespace WS2812B {
	// Data Transfer Time [units: seconds]
	static constexpr float T0H = 0.4e-6f;  // duration [sec] of "0 code, high voltage"
	static constexpr float T1H = 0.8e-6f;  // duration [sec] of "1 code, high voltage"
	static constexpr float T0L = 0.85e-6f; // duration [sec] of "0 code, low voltage"
	static constexpr float T1L = 0.45e-6f; // duration [sec] of "1 code, low voltage"
	static constexpr float T_ERROR = 150e-9f; // duration [sec] which {T0H, T1H, T0L, T1L} can deviate by.
	static constexpr float T_RES = 0.5e-6f;  // duration [sec] of "reset code (low voltage)"

	[[nodiscard]]
	static constexpr bool check_duration(float required, float tested) {
		return tested >= (required - T_ERROR) && tested <= (required + T_ERROR);
	}

	struct Color {
		uint8_t r, g, b;

		/// @return uint32_t with complemented 0s where alpha would be in RGBA
		uint32_t into_32_bit() const {
			union {
				uint32_t num;
				Color color;
				uint8_t arr[4];
			} u;

			u.color = *this;
			u.arr[3] = 0;
			return u.num;
		}

		/// @brief Modifies the brightness
		/// @param brightness 0 is black, 255 fully bright. Greater than 255 is undefined behaviour.
		void set_brightness(uint8_t brightness) {
			const auto m = max(r, max(g, b));
			r = ((uint16_t)r * brightness) / m;
			g = ((uint16_t)g * brightness) / m;
			b = ((uint16_t)b * brightness) / m;
		}
	};

	enum class TickResult {
		Ok,
		Finished
	};

	struct ColorBuffer {
		Color* start = nullptr;
		Color* end = nullptr;
	};

	/// @brief class to encode RGB data and modulate it using the PWM protocol of the WS2812B
	/// @example
	/// #include "pwm.h"
	/// bool flag_finished_transmission = false;
	/// WS2812B::Transmitter tx;
	///
	/// void main() {
	/// 	constexpr int tx_pin; // insert some output pin number here
	/// 	constexpr float time_between_interrupts = 1.0 / 16e6;
	///
	/// 	if (!tx.configure(tx_pin, time_between_interrupts)) {
	/// 		// time_between_interrupts is too big and the modulation won't work...
	/// 		// break the program
	/// 	}
	///
	///		const int len = 5;
	/// 	auto data = new WS2812B::Color[len];
	/// 	// initiate the data here...
	/// 	tx.feed({data, data+len});
	/// 	tx.start(); // must manually activate or it won't start
	///
	/// 	for(;;) {
	/// 		if (flag_finished_transmission) {
	/// 			flag_finished_transmission = false;
	/// 			tx.asynch_reset();
	/// 			tx.start();
	/// 		}
	/// 	}
	/// }
	///
	/// // this function is pseudocode for an interrupt routine
	/// void timer_ISR() {
	/// 	if (tx.tick() == WS2812B::TickResult::Finished) {
	/// 		flag_finished_transmission = true;
	/// 	}
	/// }
	class Transmitter {
	 public:
		enum class Flag: uint8_t {
			output_level = 1, // 0 for low, 1 for high
			data_bit = 2,     // the bit currently being transmitted
			resetting = 4,    // true if the transmitting a reset signal
			start = 8         // if true, when a reset signal is done, will stard sending data
		};

	 public:
		Transmitter() : buffer() {
			// TODO
		}

	 public:
		/// @brief configures the component to work according to the specified parameters.
		/// Make sure to configure BEFORE enabling the timer interrupt
		/// @param pin output port number. The pwm signal will be transmitted over this port
		/// @param tick_duration_seconds time difference in seconds between each timer interrupt
		/// @return false if the configuration failed because the tick_duration was to long 
		bool configure(int pin, float tick_duration_seconds) {
			uint32_t
				t_0_L = T0L / tick_duration_seconds,
				t_0_H = T0H / tick_duration_seconds,
				t_1_L = T1L / tick_duration_seconds,
				t_1_H = T1H / tick_duration_seconds,
				t_res = T_RES / tick_duration_seconds;

			// TODO this is not tested
			if (!check_duration(T0L, t_0_L * tick_duration_seconds))
				if (!check_duration(T0L, ++t_0_L * tick_duration_seconds))
					return false;
			if (!check_duration(T0H, t_0_H * tick_duration_seconds))
				if (!check_duration(T0H, ++t_0_H * tick_duration_seconds))
					return false;
			if (!check_duration(T1L, t_1_L * tick_duration_seconds))
				if (!check_duration(T1L, ++t_1_L * tick_duration_seconds))
					return false;
			if (!check_duration(T1H, t_1_H * tick_duration_seconds))
				if (!check_duration(T1H, ++t_1_H * tick_duration_seconds))
					return false;
			if (t_res * tick_duration_seconds < T_RES) ++t_res;

			set_ticks_required_RES(t_res);
			asynch_reset();

			set_ticks_required_0_L(t_0_L);
			set_ticks_required_0_H(t_0_H);
			set_ticks_required_1_L(t_1_L);
			set_ticks_required_1_H(t_1_H);

			pinMode(pin = pin, OUTPUT);

			return true;
		}

		/// @brief Function to be called upon a timer interrupt.
		/// The duration between the interrupts must be configured in the configure() method.
		/// @returns false if finished transmission 
		[[nodiscard]]
		TickResult tick() {
			if (++tick_counter == ticks_required) {
				if (!_get_flag(Flag::resetting)) {
					if (!_get_flag(Flag::output_level)) { // finished transmitting the bit
						increase_iterators(); // increase the data iterators
						if (is_done()) {
							start_reset();
							reset_flag(Flag::start);
							return TickResult::Finished;
						}
						read_data_bit(); // get next data bit
					}
					flip_flag(Flag::output_level);// change output level
					write_to_port(); // update the output port
					update_tick_required();
					return TickResult::Ok;
				}
				// here, Flag::resetting is 1
				if (_get_flag(Flag::start)) {
					start_data();
					return tick();
				}
			}
			return TickResult::Ok;
		}


		/// @brief give a buffer to be coded and modulated via this entity
		/// @returns the previous buffer
		[[nodiscard]]
		ColorBuffer feed(ColorBuffer &&buffer) {
			auto ret = std::exchange(this->buffer, std::move(buffer));
			asynch_reset();
			return std::move(ret);
		}

		void start() { set_flag(Flag::start); }

		void asynch_reset() {
			if (!_get_flag(Flag::resetting)) {
				start_reset();
				if (_get_flag(Flag::output_level)) {
					tick_counter = -1; // because called asynchronously, must compensate with +1 tick
				}
			}
		}

		auto get_pin_number() const { return pin; }

		/// @returns number of ticks for transmitting '0', High voltage level
		[[nodiscard]] uint32_t _get_ticks_required_0_H() const { return tick_counts[0]; };

		/// @returns number of ticks for transmitting '0', Low voltage level
		[[nodiscard]] uint32_t _get_ticks_required_0_L() const { return tick_counts[1]; };

		/// @returns number of ticks for transmitting '1', High voltage level
		[[nodiscard]] uint32_t _get_ticks_required_1_H() const { return tick_counts[2]; };

		/// @returns number of ticks for transmitting '1', Low voltage level
		[[nodiscard]] uint32_t _get_ticks_required_1_L() const { return tick_counts[3]; };

		/// @returns number of ticks for transmitting a reset signal
		[[nodiscard]] uint32_t _get_ticks_required_RES() const { return tick_counts[4]; };

		[[nodiscard]]
		bool _get_flag(Flag option) const { return flags & (uint8_t)option; }

	 private: // Private Methods

		void start_reset() {
			set_flag(Flag::resetting);
			if (_get_flag(Flag::output_level)) {
				reset_flag(Flag::output_level);
				write_to_port();
				tick_counter = 0;
			} else {
				// only extend the current counter, rather than restart it to save time
				tick_counter -= _get_ticks_required_RES() - ticks_required;
			}
			ticks_required = _get_ticks_required_RES();
		}

		void start_data() {
			reset_flag(Flag::resetting);
			ticks_required = 1;
			tick_counter = 0;
			bit_index == 23;
			data_iterator = buffer.start - 1;
		}

		/// @returns true if transmission of all data is finished
		[[nodiscard]]
		bool is_done() const { return data_iterator == buffer.end; }

		void increase_iterators() {
			if (++bit_index == 24) { // DO NOT change to ">= 24" because of restart_transmission() which enables transmission of first bit
				bit_index = 0;
				++data_iterator;
			}
		}

		void write_to_port() const { digitalWrite(pin, _get_flag(Flag::output_level)); }

		void set_ticks_required_0_L(uint32_t ticks) { tick_counts[0] = ticks; } // number of ticks for transmitting '0', Low voltage level
		void set_ticks_required_0_H(uint32_t ticks) { tick_counts[1] = ticks; } // number of ticks for transmitting '0', High voltage level
		void set_ticks_required_1_L(uint32_t ticks) { tick_counts[2] = ticks; } // number of ticks for transmitting '1', Low voltage level
		void set_ticks_required_1_H(uint32_t ticks) { tick_counts[3] = ticks; } // number of ticks for transmitting '1', High voltage level
		void set_ticks_required_RES(uint32_t ticks) { tick_counts[4] = ticks; } // number of ticks for transmitting a reset signal

		void read_data_bit() {
			write_flag(Flag::data_bit, (process_rgb(*data_iterator) >> bit_index) & 1);
		}

		void update_tick_required() { ticks_required = tick_counts[get_tick_counts_index()]; }

		/// @brief Utility for converting RGB into GRB according to the protocol, then into 32 bit.
		[[nodiscard]]
		static uint32_t process_rgb(Color c) {
			std::swap(c.r, c.g);
			return c.into_32_bit();
		}


		void write_flag(Flag option, bool value) {
			if (value) {set_flag(option);} else {reset_flag(option);}
		}

		void set_flag(Flag option) { flags |= (uint8_t)option; }

		void reset_flag(Flag option) { flags &= ~(uint8_t)option; }

		void flip_flag(Flag option) { flags ^= (uint8_t)option; }

		[[nodiscard]]
		uint8_t get_tick_counts_index() const { return flags & 3; }

	 private: // Fields
		// Configurations:
		uint32_t tick_counts[5] = {}; // { 0L, 0H, 1L, 1H, RESET }
		int pin; // hardware output port
		ColorBuffer buffer = {nullptr, nullptr}; // TODO default constrcut

		const Color *data_iterator = nullptr; // iterator over each color item
		uint32_t tick_counter = 0; // current counter of ticks until next change of pwm stage
		uint32_t ticks_required; // when tick_counter reaches this, it resets
		uint8_t bit_index = -1; // iterator over bits
		uint8_t flags = 0; // all booleans compressed into 1 byte. Ordered by the enumerations of "enum class Flag"
	};
}