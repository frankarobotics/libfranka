// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/duration.h>

namespace franka {

Duration::Duration() noexcept : duration_{0U} {}

Duration::Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept
    : duration_{duration} {}

Duration::Duration(uint64_t milliseconds) noexcept : duration_{milliseconds} {}

Duration::operator std::chrono::duration<uint64_t, std::milli>() const noexcept {
  return duration_;
}

double Duration::toSec() const noexcept {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration_).count();
}

uint64_t Duration::toMSec() const noexcept {
  return duration_.count();
}

Duration Duration::operator+(const Duration& rhs) const noexcept {
  return duration_ + rhs.duration_;
}

Duration& Duration::operator+=(const Duration& rhs) noexcept {
  duration_ += rhs.duration_;
  return *this;
}

Duration Duration::operator-(const Duration& rhs) const noexcept {
  return duration_ - rhs.duration_;
}

Duration& Duration::operator-=(const Duration& rhs) noexcept {
  duration_ -= rhs.duration_;
  return *this;
}

Duration Duration::operator*(uint64_t rhs) const noexcept {
  return duration_ * rhs;
}

Duration& Duration::operator*=(uint64_t rhs) noexcept {
  duration_ *= rhs;
  return *this;
}

uint64_t Duration::operator/(const Duration& rhs) const noexcept {
  return duration_ / rhs.duration_;
}

Duration Duration::operator/(uint64_t rhs) const noexcept {
  return duration_ / rhs;
}

Duration& Duration::operator/=(uint64_t rhs) noexcept {
  duration_ /= rhs;
  return *this;
}

Duration Duration::operator%(const Duration& rhs) const noexcept {
  return duration_ % rhs.duration_;
}

Duration Duration::operator%(uint64_t rhs) const noexcept {
  return duration_ % rhs;
}

Duration& Duration::operator%=(const Duration& rhs) noexcept {
  duration_ %= rhs.duration_;
  return *this;
}

Duration& Duration::operator%=(uint64_t rhs) noexcept {
  duration_ %= rhs;
  return *this;
}

bool Duration::operator==(const Duration& rhs) const noexcept {
  return duration_ == rhs.duration_;
}

bool Duration::operator!=(const Duration& rhs) const noexcept {
  return duration_ != rhs.duration_;
}

bool Duration::operator<(const Duration& rhs) const noexcept {
  return duration_ < rhs.duration_;
}

bool Duration::operator<=(const Duration& rhs) const noexcept {
  return duration_ <= rhs.duration_;
}

bool Duration::operator>(const Duration& rhs) const noexcept {
  return duration_ > rhs.duration_;
}

bool Duration::operator>=(const Duration& rhs) const noexcept {
  return duration_ >= rhs.duration_;
}

Duration operator*(uint64_t lhs, const Duration& rhs) noexcept {
  return rhs * lhs;
}

}  // namespace franka
