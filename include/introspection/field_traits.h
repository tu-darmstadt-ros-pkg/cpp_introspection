//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef CPP_INTROSPECTION_FIELD_TRAITS_H
#define CPP_INTROSPECTION_FIELD_TRAITS_H

#include <boost/type_traits/integral_constant.hpp>
#include <vector>
#include <boost/array.hpp>
#include <ros/message_traits.h>

namespace cpp_introspection {
namespace field_traits {

  template <typename T> struct is_array : public boost::false_type {};
  template <typename T> struct is_vector : public boost::false_type {};
  template <typename T> struct is_container : public boost::false_type {};
  template <typename T> struct value_type {
    typedef T type;
    static const char *name() { return ros::message_traits::DataType<T>::value(); }
  };
  template <typename T> struct size {
    static size_t value() { return 1; }
    static size_t value(const T&) { return 1; }
  };

  // specializations for std::vector
  template <typename T> struct is_vector< std::vector<T> > : public boost::true_type {};
  template <typename T> struct is_container< std::vector<T> > : public boost::true_type {};
  template <typename T> struct value_type< std::vector<T> > {
    typedef T type;
    static const char *name() { return value_type<T>::name(); }
  };
  template <typename T> struct size< std::vector<T> > {
    static size_t value() { return 0; }
    static size_t value(const std::vector<T>& x) { return x.size(); }
  };

  // specializations for boost::array
  template <typename T, std::size_t N> struct is_array< boost::array<T,N> > : public boost::true_type {};
  template <typename T, std::size_t N> struct is_container< boost::array<T,N> > : public boost::true_type {};
  template <typename T, std::size_t N> struct value_type< boost::array<T,N> > {
    typedef T type;
    static const char *name() { return value_type<T>::name(); }
  };
  template <typename T, std::size_t N> struct size< boost::array<T,N> > {
    static size_t value() { return N; }
    static size_t value(const boost::array<T,N>& x) { return x.size(); }
  };

  // simple type traits
#define INTROSPECTION_DECLARE_SIMPLE_TRAITS(_type, _name) \
  template <> struct value_type< _type > { \
    typedef _type type; \
    static const char *name() { return _name; } \
  }

  INTROSPECTION_DECLARE_SIMPLE_TRAITS(bool, "bool");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(uint8_t, "uint8");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(int8_t, "int8");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(uint16_t, "uint8");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(int16_t, "int16");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(uint32_t, "uint32");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(int32_t, "int32");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(uint64_t, "uint64");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(int64_t, "int64");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(float, "float32");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(double, "float64");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(ros::Time, "time");
  INTROSPECTION_DECLARE_SIMPLE_TRAITS(ros::Duration, "duration");

}} // namespace

#endif // CPP_INTROSPECTION_FIELD_TRAITS_H
