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

#ifndef CPP_INTROSPECTION_FIELD_H
#define CPP_INTROSPECTION_FIELD_H

#include <introspection/forwards.h>
#include <introspection/type.h>
#include <introspection/field.h>

namespace cpp_introspection {

  class Accessor {};
  class ConstAccessor {};

  class Field
  {
  private:
    const Message& message_;

  public:
    Field(const Message& message) : message_(message) { }
    virtual ~Field() { }

    const Message& message() const { return message_; }

    virtual const char* getName() const = 0;
    virtual const char* getDataType() const = 0;
    virtual const char* getValueType() const = 0;
    virtual std::size_t getIndex() const = 0;
    virtual std::size_t getSize() const = 0;
    virtual const std::type_info& getTypeId() const = 0;
    virtual TypePtr getType() const { return type(getValueType()); }

    virtual bool isArray() const = 0;
    virtual bool isVector() const = 0;
    virtual bool isContainer() const = 0;
    virtual bool isSimple() const = 0;
    virtual bool isFixedSize() const = 0;
    virtual bool isMessage() const = 0;

    virtual bool hasInstance() const { return false; }
    virtual boost::any get(std::size_t i = 0) const { return boost::any(); }
    template <typename T> T as(std::size_t i = 0) const { return getType()->as<T>(get(i)); }

    virtual void setAny(const boost::any& value, std::size_t = 0) const { }
    template <typename T> void set(const T& value, std::size_t i = 0) const { setAny(getType()->from(value), i); }

    // capacity
    virtual std::size_t size() const { return 1; }
    virtual void resize(size_t size) const {}
    virtual bool empty() const { return false; }
    virtual std::size_t max_size() const { return 1; }

    virtual MessagePtr expand(std::size_t i = 0) const;
    virtual FieldPtr access(Accessor& accessor) const = 0;
    virtual FieldPtr access(ConstAccessor& accessor) const = 0;
  };

} // namespace cpp_introspection

#endif // CPP_INTROSPECTION_FIELD_H
