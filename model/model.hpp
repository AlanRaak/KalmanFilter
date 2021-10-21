#pragma once

#include "../math/matrix.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <iostream>

namespace model
{
	struct rig
	{
		const double wheelbase;
		const double width;
		const double rear_bumper;
		const double front_bumper;
		const double max_throttle;
		const double max_brake;
		const double max_abs_steering_rate;
		const double steering_velocity;
		const double max_abs_steering_angle;
		const double throttle_force;
		const double brake_force;
		const double drag;
		const double rolling_resistance;

		rig(double wheelbase = 0.90,
			double width = 0.67,
			double rear_bumper = 0.3,
			double front_bumper = 1.35,
			double max_throttle = 1.0,
			double max_brake = 1.0,
			double max_abs_steering_rate = 1,
			double steering_velocity = 1,
			double max_abs_steering_angle = 0.6,
			double throttle_force = 2.0,
			double brake_force = 6.0,
			double drag = 0.03,
			double rolling_resistance = 0.5) :
			wheelbase(wheelbase),
			width(width),
			rear_bumper(rear_bumper),
			front_bumper(front_bumper),
			max_throttle(max_throttle),
			max_brake(max_brake),
			max_abs_steering_rate(max_abs_steering_rate),
			steering_velocity(steering_velocity),
			max_abs_steering_angle(max_abs_steering_angle),
			throttle_force(throttle_force),
			brake_force(brake_force),
			drag(drag),
			rolling_resistance(rolling_resistance) {}
	};

	enum class gear
	{
		forward,
		reverse
	};

	struct position
	{
		double x = 0;
		double y = 0;
	};

	struct control_signal
	{
		double throttle = 0;
		double brake = 0;
		double steering_rate = 0;
		enum gear gear = gear::forward;
	};

	struct state{
		position pos{.x=0, .y=0};
		double orientation = 0;
		double steering_angle = 0;
		double speed = 0;
		enum gear gear = gear::forward;
	};


	inline std::ostream& operator << (std::ostream &os, const state &s) {
		return (os << "State [Pos: (" << s.pos.x << ";" << s.pos.y << "), Orient: " << s.orientation << ", Steer: " << s.steering_angle << ", Speed: " << s.speed << ", Gear: " << static_cast<int>(s.gear) << "] " << std::endl);
	}

	inline std::ostream& operator << (std::ostream &os, const control_signal &c) {
		return (os << "Ctrls [Thr: " << c.throttle << ", Brk: " << c.brake << ", Steer: " << c.steering_rate << ", Gear: " << static_cast<int>(c.gear) << "] "  << std::endl);
	}

	const auto check_control_constraints{[] (const control_signal& u, const rig& rig)
	{
		if (rig.max_abs_steering_rate < std::fabs(u.steering_rate))
		{
			throw std::runtime_error{"Maximum allowed steering rate exceeded"};
		}
		if (u.throttle < 0.0 || 1.0 < u.throttle)
		{
			throw std::runtime_error{"Throttle control out of allowed boundaries"};
		}
		if (u.brake < 0.0 || 1.0 < u.brake)
		{
			throw std::runtime_error{"Brake control out of allowed boundaries"};
		}
		if (0.0 < u.brake && 0.0 < u.throttle)
		{
			throw std::runtime_error{"Brake and torque simultaneous control is not allowed"};
		}
	}};



	// std::atomic<bool> running{true};

	// [[nodiscard]] inline bool is_running()
	// {
	// 	return running.load(std::memory_order_relaxed);
	// }

	// inline void close()
	// {
	// 	running.store(false, std::memory_order_relaxed);
	// }

	// inline state next_state(state x, const float dt)
	inline math::matrix next_state(math::matrix x, const float& dt)
	{
		// x is probably -> 0:x_pos, 1:y_pos, 2:orientation, 3:steering_angle, 4:speed

		constexpr double dt_divider{10};
		constexpr double wheelbase{0.9};
		const double ddt{dt/dt_divider};

		for (int i{}; i < dt_divider; ++i)
		{
			// Change position
			x.array[0][0] += x.array[4][0] * std::cos(x.array[2][0]) * ddt;
			x.array[1][0] += x.array[4][0] * std::sin(x.array[2][0]) * ddt;

			// Change orientation
			x.array[2][0] += x.array[4][0] * std::tan(x.array[3][0]) / wheelbase * ddt;
		}
		// Normalize orientation
		x.array[2][0] = std::atan2(std::sin(x.array[2][0]), std::cos(x.array[2][0]));

		return x;
	}

	inline math::matrix noisify_state(math::matrix state)
	{
		state.array[0][0] += (rand() % 101 - 100 / 2.0) / 100.0; // pos_X
		state.array[1][0] += (rand() % 101 - 100 / 2.0) / 100.0; // pos_y
		state.array[2][0] += (rand() % 11 - 10 / 2.0) / 100.0; // orientation
		state.array[3][0] += (rand() % 11 - 10 / 2.0) / 100.0; // steering_angle
		state.array[4][0] += (rand() % 101 - 100 / 2.0) / 100.0; // speed

		return state;
	}
}
