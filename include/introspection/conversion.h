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

#ifndef CPP_INTROSPECTION_CONVERSION_H
#define CPP_INTROSPECTION_CONVERSION_H

#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <introspection/type.h>

namespace cpp_introspection {

  template <
    typename OtherT,
    typename Enabled = void
  >
  struct converter {
    static OtherT to(const Type& type, const boost::any& value)   { return OtherT(); }
    static boost::any from(const Type& type, const OtherT& value) { return boost::any(); }
  };

  // floating_point
  template <typename OtherT>
  struct converter<OtherT, typename boost::enable_if<boost::is_floating_point<OtherT> >::type>
  {
    static OtherT to(const Type& type, const boost::any& value)   { return OtherT(type.as_double(value)); }
    static boost::any from(const Type& type, const OtherT& value) { return type.from_double(value); }
  };

  // signed
  template <typename OtherT>
  struct converter<OtherT, typename boost::enable_if<boost::is_signed<OtherT> >::type> {
    static OtherT to(const Type& type, const boost::any& value)   { return OtherT(type.as_int(value)); }
    static boost::any from(const Type& type, const OtherT& value) { return type.from_int(value); }
  };

  // unsigned
  template <typename OtherT>
  struct converter<OtherT, typename boost::enable_if<boost::is_unsigned<OtherT> >::type> {
    static OtherT to(const Type& type, const boost::any& value)   { return OtherT(type.as_unsigned(value)); }
    static boost::any from(const Type& type, const OtherT& value) { return type.from_unsigned(value); }
  };

  // string
  template <>
  struct converter<std::string> {
    static std::string to(const Type& type, const boost::any& value)   { return type.as_string(value); }
    static boost::any from(const Type& type, const std::string& value) { return type.from_string(value); }
  };

  template <typename TargetType> TargetType Type::as(const boost::any& value) const   { return converter<TargetType>::to(*this, value); }
  template <typename SourceType> boost::any Type::from(const SourceType& value) const { return converter<SourceType>::from(*this, value); }

} // namespace cpp_introspection

#endif // CPP_INTROSPECTION_CONVERSION_H
