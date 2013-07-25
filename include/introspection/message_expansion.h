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

#ifndef CPP_INTROSPECTION_MESSAGE_EXPANSION_H
#define CPP_INTROSPECTION_MESSAGE_EXPANSION_H

#include <introspection/message.h>

namespace cpp_introspection {

class MessageForwarder : public Message {
protected:
  const MessagePtr& parent_;

public:
  MessageForwarder(const MessagePtr& parent) : parent_(parent) {}
  virtual ~MessageForwarder() {}

  virtual PackagePtr package() const { return parent_->package(); }
  virtual const char* getPackageName() const { return parent_->getPackageName(); }

  virtual const char* getName() const { return parent_->getName(); }
  virtual const char* getDataType() const { return parent_->getDataType(); }
  virtual const char* getMD5Sum() const { return parent_->getMD5Sum(); }
  virtual const char* getDefinition() const { return parent_->getDefinition(); }
  virtual const std::type_info& getTypeId() const { return parent_->getTypeId(); }

  virtual bool isMessage() const { return parent_->isMessage(); }
  virtual bool isSimple() const { return parent_->isSimple(); }
  virtual bool isFixedSize() const { return parent_->isFixedSize(); }
  virtual bool hasHeader() const { return parent_->hasHeader(); }

  virtual std_msgs::Header* getHeader(const VoidPtr& instance) const { return parent_->getHeader(instance); }
  virtual const std_msgs::Header* getHeader(const VoidConstPtr& instance) const { return parent_->getHeader(instance); }
  virtual std::string* getFrameId(const VoidPtr& instance) const { return parent_->getFrameId(instance); }
  virtual const std::string* getFrameId(const VoidConstPtr& instance) const { return parent_->getFrameId(instance); }
  virtual ros::Time* getTimeStamp(const VoidPtr& instance) const { return parent_->getTimeStamp(instance); }
  virtual const ros::Time* getTimeStamp(const VoidConstPtr& instance) const { return parent_->getTimeStamp(instance); }

  virtual VoidPtr createInstance() const { return parent_->createInstance(); }
  virtual void serialize(ros::serialization::OStream& stream, const VoidConstPtr& instance = VoidConstPtr()) const { parent_->serialize(stream, instance); }
  virtual ros::SerializedMessage serialize(const VoidConstPtr& instance = VoidConstPtr()) const { return parent_->serialize(instance); }
  virtual std::size_t serializationLength(const VoidConstPtr& instance = VoidConstPtr()) const { return parent_->serializationLength(instance); }
  virtual VoidPtr deserialize(ros::serialization::IStream& stream, const VoidPtr& instance = VoidPtr()) const { return parent_->deserialize(stream, instance); }

  virtual bool hasInstance() const { return parent_->hasInstance(); }
  virtual VoidPtr getInstance() const { return parent_->getInstance(); }
  virtual VoidConstPtr getConstInstance() const { return parent_->getConstInstance(); }

  virtual MessagePtr introspect(const VoidPtr& instance) const { return parent_->introspect(instance); }
  virtual MessagePtr introspect(void *instance) const { return parent_->introspect(instance); }
  virtual MessagePtr introspect(const VoidConstPtr& instance) const { return parent_->introspect(instance); }
  virtual MessagePtr introspect(void const *instance) const { return parent_->introspect(instance); }
};

class ExpandedMessage : public MessageForwarder {
private:
  V_Field fields_;
  M_Field fields_by_name_;
  V_FieldName field_names_;

  std::string separator_;
  std::string prefix_;

public:
  ExpandedMessage(const MessagePtr& parent, const std::string &separator = ".", const std::string &prefix = std::string())
    : MessageForwarder(parent)
    , separator_(separator)
    , prefix_(prefix)
  {
    expand(parent, prefix);
  }
  virtual ~ExpandedMessage() {}

  virtual const V_Field& fields() const { return fields_; }
  virtual FieldWPtr field(const std::string& name) const { return fields_by_name_.at(name); }
  virtual const V_FieldName& getFieldNames() const { return field_names_; }

  virtual MessagePtr introspect(void *instance) const { return MessagePtr(new ExpandedMessage(parent_->introspect(instance), separator_, prefix_)); }
  virtual MessagePtr introspect(void const *instance) const { return MessagePtr(new ExpandedMessage(parent_->introspect(instance), separator_, prefix_)); }

private:
  void expand(const MessagePtr& message, const std::string& prefix);
};

class ExpandedField : public Field {
private:
  const Field& parent_;
  std::size_t index_;
  std::string name_;

public:
  ExpandedField(const Field& parent, std::string name = std::string(), std::size_t index = 0)
    : parent_(parent), index_(index), name_(!name.empty() ? name : parent_.getName())
  {}
  virtual ~ExpandedField() {}

  virtual const Message& message() const { return parent_.message(); }

  virtual const char* getName() const { return name_.c_str(); }
  virtual const char* getDataType() const { return parent_.getValueType(); }
  virtual const char* getValueType() const { return parent_.getValueType(); }
  virtual std::size_t getIndex() const { return parent_.getIndex(); }
  virtual const std::type_info& getTypeId() const { return parent_.getTypeId(); }
  virtual TypePtr getType() const { return parent_.getType(); }

  virtual bool isArray() const { return false; }
  virtual bool isVector() const { return false; }
  virtual bool isContainer() const { return false; }
  virtual bool isSimple() const { return parent_.isSimple(); }
  virtual bool isFixedSize() const { return parent_.isFixedSize(); }
  virtual bool isMessage() const { return parent_.isMessage(); }

  virtual bool hasInstance() const { return parent_.hasInstance(); }
  virtual boost::any get(std::size_t i = 0) const { return parent_.get(index_); }
  virtual void setAny(const boost::any& value, std::size_t i = 0) const { parent_.setAny(value, index_); }

  virtual FieldPtr access(AccessorBase& accessor) const { return FieldPtr(); }
};

} // namespace cpp_introspection

#endif // CPP_INTROSPECTION_MESSAGE_EXPANSION_H
