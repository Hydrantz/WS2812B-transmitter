#pragma once

#include <stdint.h>
#include <arduino.h>

namespace WS2812B {
	// Data Transfer Time [units: seconds]
	static constexpr float T0H = 0.4e-6f;  // 0 code, high voltage time
	static constexpr float T1H = 0.8e-6f;  // 1 code, high voltage time
	static constexpr float T0L = 0.85e-6f; // 0 code, low voltage time
	static constexpr float T1L = 0.45e-6f; // 1 code, low voltage time
	static constexpr float TRES = 0.5e-6f;  // low voltage time


	struct Color {
		uint8_t r;
		uint8_t g;
		uint8_t b;

		/// @return uint32_t with complemented 0s where alpha would be in RGBA
		uint32_t as_32_bit() const {
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
		/// @param brightness 0 is completely dark, 255 is not modified
		void dim(uint8_t brightness) {
			r = ((uint16_t)r * brightness) / 255;
			g = ((uint16_t)g * brightness) / 255;
			b = ((uint16_t)b * brightness) / 255;
		}

		/// @brief Makes the color more vivid?
		void normalize() {
			const auto max = std::max(r, std::max(g, b));
			r = ((uint16_t)r * 255) / max;
			g = ((uint16_t)g * 255) / max;
			b = ((uint16_t)b * 255) / max;
		}
	};

	enum class TickResult {
		Disabled,
		Ok,
		Finished
	};

	struct ColorBuffer {
		Color* start = nullptr, end = nullptr;
	};

	/// @brief class to encode RGB data and modulate it using the PWM protocol of the WS2812B
	/// @example
	/// bool flag_finished_transmission = false;
	/// WS2812B::Transmitter pwm_tx;
	///
	/// void main() {
	/// 	constexpr int pwm_pin; // insert some output pin number here
	/// 	constexpr float time_between_interrupts = 1.0 / 16e6;
	///
	/// 	if (!pwm_tx.configure(pwm_pin, time_between_interrupts)) {
	/// 		// time_between_interrupts is too big and the modulation won't work...
	/// 		// break the program
	/// 	}
	///
	/// 	Color *data = new Color[5];
	/// 	// initiate the data here...
	/// 	pwm_tx.feed({data, data+5});
	/// 	pwm_tx.set_active(true); // must manually activate or it won't start
	///
	/// 	for(;;) {
	/// 		if (flag_finished_transmission) {
	/// 			flag_finished_transmission = false;
	/// 			pwm_tx.restart_transmission();
	/// 			pwm_tx.set_active(true);
	/// 		}
	/// 	}
	/// }
	///
	/// // this function is pseudocode for an interrupt routine
	/// void timer_ISR() {
	/// 	if (pwm_tx.tick() == WS2812B::TickResult::Finished)
	/// 		flag_finished_transmission = true;
	/// 	}
	/// }
	class Transmitter {
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
				t_res = TRES / tick_duration_seconds;

			// todo better check
			if (!ticks_required_0_H || !ticks_required_0_L || !ticks_required_1_H || !ticks_required_1_L || !t_res) {
				return false;
			}

			set_active(false);

			set_ticks_required_0_L(t_0_L);
			set_ticks_required_0_H(t_0_H);
			set_ticks_required_1_L(t_1_L);
			set_ticks_required_1_H(t_1_H);
			set_ticks_required_RES(t_res);

			pinMode(pin = pin, OUTPUT);

			restart_transmission();
			return true;
		}

		/// @brief Function to be called upon a timer interrupt.
		/// The duration between the interrupts must be configured in the configure() method.
		/// @returns false if finished transmission 
		[[nodiscard]]
		TickResult tick() {
			// safety measures
			if (!is_active()) return TickResult::Disabled;
			if (is_done()) {
				set_active(false);
				return TickResult::Finished;
			}
			if (--current_tick_counter == 0) {
				if (current_output_level == false) { // finished transmitting the bit
					increase_iterators(); // increase the data iterators
					if (is_done()) {
						set_active(false);
						return TickResult::Finished;
					}
					read_bit(); // get next data bit
				}
				current_output_level = !current_output_level; // change output level
				write_to_port(); // update the output port
				update_tick_counter();
			}
			return TickResult::Ok;
		}

		[[nodiscard]]
		bool is_active() const {} { return enable; }
		void set_active(bool enable) { this->enable = enable; }

		void restart_transmission() {
			set_active(false);

			data_iterator = buffer.start;

			// the next three lines make sure that the first bit is sent properly on the next tick
			current_tick_counter = 1;
			current_output_level = false; // as if a bit had just finished transmitting
			bit_index = -1;
		}

		/// @brief give a buffer to be coded and modulated via this entity
		/// @returns the previous buffer
		[[nodiscard]]
		ColorBuffer&& feed(ColorBuffer &&buffer) {
			set_active(false);
			auto ret = std::exchange(this->buffer, std::move(buffer));
			restart_transmission();
			return ret;
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

	 private: // Private Methods

		/// @returns true if transmission of all data is finished
		[[nodiscard]]
		bool is_done() const { return is_active() && data_iterator == buffer.end; }

		void increase_iterators() {
			if (++bit_index == 24) { // DO NOT change to ">= 24" because of restart_transmission() which enables transmission of first bit
				bit_index = 0;
				++data_iterator;
			}
		}

		void write_to_port() const { digitalWrite(pin, current_output_level); }

		void set_ticks_required_0_L(uint32_t ticks) { tick_counts[0] = ticks; } // number of ticks for transmitting '0', Low voltage level
		void set_ticks_required_0_H(uint32_t ticks) { tick_counts[1] = ticks; } // number of ticks for transmitting '0', High voltage level
		void set_ticks_required_1_L(uint32_t ticks) { tick_counts[2] = ticks; } // number of ticks for transmitting '1', Low voltage level
		void set_ticks_required_1_H(uint32_t ticks) { tick_counts[3] = ticks; } // number of ticks for transmitting '1', High voltage level
		void set_ticks_required_RES(uint32_t ticks) { tick_counts[4] = ticks; } // number of ticks for transmitting a reset signal

		void read_bit() { current_bit_value = (data_iterator->as_32_bit() >> bit_index) & 1; }

		void update_tick_counter() { current_tick_counter = tick_counts[(uint8_t)current_bit_value * 2 + current_output_level]; }

	 private: // Fields
		// Configurations:
		uint32_t tick_counts[5] = {}; // { 0L, 0H, 1L, 1H, RESET }
		int pin; // hardware output port
		ColorBuffer buffer = {nullptr, nullptr}; // TODO default constrcut

		bool enable = false;
		bool flag_reset_occured; // TODO ?
		uint32_t current_tick_counter = 0; // current counter of ticks until next change of pwm stage
		bool current_bit_value = false; // the bit currently being transmitted
		bool current_output_level = false; // 0 for low, 1 for high
		uint8_t bit_index = -1; // iterator over bits
		const Color *data_iterator = nullptr; // iterator over each color item
	};
}