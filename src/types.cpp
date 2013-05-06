//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#include <introspection/type.h>

namespace cpp_introspection {

  static V_Type g_types;
  static M_Type g_types_by_name;

  namespace {
    static Type::StaticInitializer ros_bool    (TypePtr(new BoolType("bool")));
    static Type::StaticInitializer ros_int8    (TypePtr(new NumericType<int8_t>("int8")));
    static Type::StaticInitializer ros_uint8   (TypePtr(new NumericType<uint8_t>("uint8")));
    static Type::StaticInitializer ros_int16   (TypePtr(new NumericType<int16_t>("int16")));
    static Type::StaticInitializer ros_uint16  (TypePtr(new NumericType<uint16_t>("uint16")));
    static Type::StaticInitializer ros_int32   (TypePtr(new NumericType<int32_t>("int32")));
    static Type::StaticInitializer ros_uint32  (TypePtr(new NumericType<uint32_t>("uint32")));
    static Type::StaticInitializer ros_int64   (TypePtr(new NumericType<int64_t>("int64")));
    static Type::StaticInitializer ros_uint64  (TypePtr(new NumericType<uint64_t>("uint64")));
    static Type::StaticInitializer ros_float32 (TypePtr(new NumericType<float>("float32")));
    static Type::StaticInitializer ros_float64 (TypePtr(new NumericType<double>("float64")));
    static Type::StaticInitializer ros_string  (TypePtr(new StringType("string")));
    static Type::StaticInitializer ros_time    (TypePtr(new TimeType("time")));
    static Type::StaticInitializer ros_duration(TypePtr(new DurationType("duration")));
    static Type::StaticInitializer ros_byte    (TypePtr(new NumericType<int8_t>("byte")));
    static Type::StaticInitializer ros_char    (TypePtr(new NumericType<uint8_t>("char")));
  } // namespace

  const TypePtr& Type::add(const TypePtr& type, const std::string& alias)
  {
    if (alias.empty()) {
      g_types.push_back(type);
      g_types_by_name[type->getName()] = type;
    } else {
      g_types_by_name[alias] = type;
    }
    return type;
  }

  Type::StaticInitializer::StaticInitializer(const TypePtr& type)
  {
    Type::add(type);
  }

  TypePtr Type::alias(const std::string& name) const
  {
    return add(shared_from_this(), name);
  }

  TypePtr UnknownType::Instance() {
    static TypePtr s_instance;
    if (!s_instance) s_instance.reset(new UnknownType);
    return s_instance;
  }

  TypePtr type(const std::string& name)
  {
    if (!g_types_by_name.count(name)) return UnknownType::Instance();
    return g_types_by_name[name].lock();
  }

} // namespace cpp_introspection
