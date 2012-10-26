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

#ifndef CPP_INTROSPECTION_MESSAGE_TEMPLATE_H
#define CPP_INTROSPECTION_MESSAGE_TEMPLATE_H

#include <introspection/message.h>
#include <ros/message_traits.h>

#include <cstring>

namespace cpp_introspection {

  template <typename T>
  class MessageTemplate : public Message {
  public:
    typedef T MessageType;

    virtual ~MessageTemplate() {}

    const char* getDataType() const    { return ros::message_traits::DataType<MessageType>::value(); }
    const char* getMD5Sum() const      { return ros::message_traits::MD5Sum<MessageType>::value(); }
    const char* getDefinition() const  { return ros::message_traits::Definition<MessageType>::value(); }
    const std::type_info& getTypeId() const { return typeid(MessageType); }

    bool isSimple() const              { return ros::message_traits::isSimple<MessageType>(); }
    bool isFixedSize() const           { return ros::message_traits::isFixedSize<MessageType>(); }
    bool hasHeader() const             { return ros::message_traits::hasHeader<MessageType>(); }

    ::std_msgs::Header* getHeader(const VoidPtr& instance) const;
    const ::std_msgs::Header* getHeader(const VoidConstPtr& instance) const;
    std::string* getFrameId(const VoidPtr& instance) const;
    const std::string* getFrameId(const VoidConstPtr& instance) const;
    ros::Time* getTimeStamp(const VoidPtr& instance) const;
    const ros::Time* getTimeStamp(const VoidConstPtr& instance) const;

    VoidPtr createInstance() const;
    void serialize(ros::serialization::OStream& stream, const VoidConstPtr& instance = VoidConstPtr()) const;
    ros::SerializedMessage serialize(const VoidConstPtr& instance = VoidConstPtr()) const;
    std::size_t serializationLength(const VoidConstPtr& instance = VoidConstPtr()) const;
    VoidPtr deserialize(ros::serialization::IStream& stream, const VoidPtr& instance = VoidPtr()) const;

    MessagePtr introspect(const VoidPtr& instance) const;
    MessagePtr introspect(void *instance) const;
    MessagePtr introspect(const VoidConstPtr& instance) const;
    MessagePtr introspect(void const *instance) const;
  };

  template <typename T>
  ::std_msgs::Header* MessageTemplate<T>::getHeader(const VoidPtr& instance) const {
    MessageType* x = static_cast<MessageType*>(instance.get());
    if (!x) return 0;
    return ros::message_traits::Header<MessageType>::pointer(*x);
  }

  template <typename T>
  const ::std_msgs::Header* MessageTemplate<T>::getHeader(const VoidConstPtr& instance) const {
    const MessageType* x = static_cast<const MessageType*>(instance.get());
    if (!x) return 0;
    return ros::message_traits::Header<MessageType>::pointer(*x);
  }

  template <typename T>
  std::string* MessageTemplate<T>::getFrameId(const VoidPtr& instance) const {
    MessageType* x = static_cast<MessageType*>(instance.get());
    if (!x) return 0;
    return ros::message_traits::FrameId<MessageType>::pointer(*x);
  }

  template <typename T>
  const std::string* MessageTemplate<T>::getFrameId(const VoidConstPtr& instance) const {
    const MessageType* x = static_cast<const MessageType*>(instance.get());
    if (!x) return 0;
    return ros::message_traits::FrameId<MessageType>::pointer(*x);
  }

  template <typename T>
  ros::Time* MessageTemplate<T>::getTimeStamp(const VoidPtr& instance) const {
    MessageType* x = static_cast<MessageType*>(instance.get());
    if (!x) return 0;
    return ros::message_traits::TimeStamp<MessageType>::pointer(*x);
  }

  template <typename T>
  const ros::Time* MessageTemplate<T>::getTimeStamp(const VoidConstPtr& instance) const {
    const MessageType* x = static_cast<const MessageType*>(instance.get());
    if (!x) return 0;
    return ros::message_traits::TimeStamp<MessageType>::pointer(*x);
  }

  template <typename T>
  VoidPtr MessageTemplate<T>::createInstance() const
  {
    return VoidPtr(new MessageType());
  }

  template <typename T>
  void MessageTemplate<T>::serialize(ros::serialization::OStream& stream, const VoidConstPtr& instance) const
  {
    const MessageType* x = static_cast<const MessageType*>(instance.get());
    if (!x) return;
    ros::serialization::serialize(stream, *x);
  }

  template <typename T>
  ros::SerializedMessage MessageTemplate<T>::serialize(const VoidConstPtr& instance) const
  {
    const MessageType* x = static_cast<const MessageType*>(instance.get());
    if (!x) return ros::SerializedMessage();
    return ros::serialization::serializeMessage<T>(*x);
  }

  template <typename T>
  std::size_t MessageTemplate<T>::serializationLength(const VoidConstPtr& instance) const
  {
    const MessageType* x = static_cast<const MessageType*>(instance.get());
    if (!x) return 0;
    return ros::serialization::serializationLength(*x);
  }

  template <typename T>
  VoidPtr MessageTemplate<T>::deserialize(ros::serialization::IStream& stream, const VoidPtr& instance) const
  {
    typename MessageType::Ptr x = boost::shared_static_cast<MessageType>(instance);
    if (!x) x = boost::static_pointer_cast<MessageType>(createInstance());
    ros::serialization::deserialize(stream, *x);
    return x;
  }

} // namespace

#endif // CPP_INTROSPECTION_MESSAGE_TEMPLATE_H
