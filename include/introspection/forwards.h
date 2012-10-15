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

#ifndef CPP_INTROSPECTION_FORWARDS_H
#define CPP_INTROSPECTION_FORWARDS_H

#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
#include <map>
#include <vector>

namespace cpp_introspection {

  class Package;
  class Message;
  class Field;
  class Type;

  typedef boost::shared_ptr<Package const> PackagePtr;
  typedef boost::weak_ptr<Package const> PackageWPtr;
  typedef boost::shared_ptr<Message const> MessagePtr;
  typedef boost::weak_ptr<Message const> MessageWPtr;
  typedef boost::shared_ptr<Field const> FieldPtr;
  typedef boost::weak_ptr<Field const> FieldWPtr;
  typedef boost::shared_ptr<Type const> TypePtr;
  typedef boost::weak_ptr<Type const> TypeWPtr;

  typedef std::vector<PackagePtr> V_Package;
  typedef std::vector<MessagePtr> V_Message;
  typedef std::vector<FieldPtr> V_Field;
  typedef std::vector<TypePtr> V_Type;

  typedef std::map<std::string,PackageWPtr> M_Package;
  typedef std::map<std::string,MessageWPtr> M_Message;
  typedef std::map<std::string,FieldWPtr> M_Field;
  typedef std::map<std::string,TypeWPtr> M_Type;

  struct CompareTypeInfo { bool operator()(const std::type_info *t1, const std::type_info *t2) { return (*t1).before(*t2); } };
  typedef std::map<const std::type_info *,MessageWPtr,CompareTypeInfo> M_TypeInfo_Message;

  typedef boost::shared_ptr<void> VoidPtr;
  typedef boost::weak_ptr<void> VoidWPtr;
  typedef boost::shared_ptr<void const> VoidConstPtr;
  typedef boost::weak_ptr<void const> VoidConstWPtr;

  typedef std::vector<std::string> V_string;

} // namespace cpp_introspection

#endif // CPP_INTROSPECTION_FORWARDS_H
