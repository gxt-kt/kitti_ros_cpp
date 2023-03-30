/**
 * @file debugstream.h
 * @author gxt_kt (gxt_kt@163.com)
 * @brief  If you have use the qDebuf() function of "Qt", you must use this module easily.
 * And the qDebug is change name to gDebug here. The detail see the "@attention".
 * The github address is https://github.com/gxt-kt/gDebugV2.0
 * @version 0.31
 * @date 2023-03-27
 *
 * @copyright Copyright (c) 2022
 *
 * @attention You can use the default gDebug/gDebug() function to output the debug stream.
 * Such as gDebug("hello world"); , Complexly you can write like gDebug("hello") << "world"; and so on.
 * And the default gDebug/gDebug() has enable the space and newline.
 * If you use the class DebugStream create a new instantiation.The space funciton is exist but the
 * auto newline is invalid.
 */

#ifndef DEBUGSTREAM_H__
#define DEBUGSTREAM_H__

#include <string>
#include <functional>
#include <cstdarg>
#include <cstring>
#include <cstdio>

#include "ros/ros.h"

// There will no debug stream output if define the NO_DEBUG_OUTPUT
//#define NO_DEBUG_OUTPUT

// The default call back function : use the printf to send the stream
inline void DebugSendStringCallBack_Default_(const char *str, int num) {
  // uint64_t aa=ros::Time::now().toNSec();
  printf("%.*s",num,str);
  // ROS_INFO("%.*s",num,str);
}

// Set the endl compatible with std::endl;
class DebugStreamEndl {};
inline void endl(DebugStreamEndl){};

//--------------------------------------------------
#define DEBUG_STREAM_COLOR_FG_NORMAL  "\x1B[0m";
#define DEBUG_STREAM_COLOR_FG_RED     "\x1b[31m";
#define DEBUG_STREAM_COLOR_FG_GREEN   "\x1B[32m";
#define DEBUG_STREAM_COLOR_FG_YELLOW  "\x1B[33m";
#define DEBUG_STREAM_COLOR_FG_BLUE    "\x1B[34m";
#define DEBUG_STREAM_COLOR_FG_MAGENTA "\x1B[35m";
#define DEBUG_STREAM_COLOR_FG_CYAN    "\x1B[36m";
#define DEBUG_STREAM_COLOR_FG_WHITE   "\x1B[37m";
//--------------------------------------------------
#define DEBUG_STREAM_COLOR_BG_NORMAL  "\x1B[49m";
#define DEBUG_STREAM_COLOR_BG_RED     "\x1b[41m";
#define DEBUG_STREAM_COLOR_BG_GREEN   "\x1B[42m";
#define DEBUG_STREAM_COLOR_BG_YELLOW  "\x1B[43m";
#define DEBUG_STREAM_COLOR_BG_BLUE    "\x1B[44m";
#define DEBUG_STREAM_COLOR_BG_MAGENTA "\x1B[45m";
#define DEBUG_STREAM_COLOR_BG_CYAN    "\x1B[46m";
#define DEBUG_STREAM_COLOR_BG_WHITE   "\x1B[47m";
//--------------------------------------------------
class normal_fg_t {}; inline void normal_fg (normal_fg_t ) {}
class red_fg_t    {}; inline void red_fg    (red_fg_t    ) {}
class green_fg_t  {}; inline void green_fg  (green_fg_t  ) {}
class yellow_fg_t {}; inline void yellow_fg (yellow_fg_t ) {}
class blue_fg_t   {}; inline void blue_fg   (blue_fg_t   ) {}
class magenta_fg_t{}; inline void magenta_fg(magenta_fg_t) {}
class cyan_fg_t   {}; inline void cyan_fg   (cyan_fg_t   ) {}
class white_fg_t  {}; inline void white_fg  (white_fg_t  ) {}
//--------------------------------------------------
class normal_bg_t {}; inline void normal_bg (normal_bg_t ) {}
class red_bg_t    {}; inline void red_bg    (red_bg_t    ) {}
class green_bg_t  {}; inline void green_bg  (green_bg_t  ) {}
class yellow_bg_t {}; inline void yellow_bg (yellow_bg_t ) {}
class blue_bg_t   {}; inline void blue_bg   (blue_bg_t   ) {}
class magenta_bg_t{}; inline void magenta_bg(magenta_bg_t) {}
class cyan_bg_t   {}; inline void cyan_bg   (cyan_bg_t   ) {}
class white_bg_t  {}; inline void white_bg  (white_bg_t  ) {}

// set the debug color
class debug_general_t     {}; inline void GENERAL     (debug_general_t)     {}
class debug_status_t      {}; inline void STATUS      (debug_status_t)      {}
class debug_warning_t     {}; inline void WARNING     (debug_warning_t)     {}
class debug_error_t       {}; inline void ERROR       (debug_error_t)       {}
class debug_fatal_error_t {}; inline void FATAL_ERROR (debug_fatal_error_t) {}


class DebugStream {
 public:
  //======================================
  explicit DebugStream \
  (std::function<void(const char*,int)>  fun_ = DebugSendStringCallBack_Default_, \
  int buf_len_ = 256);
  DebugStream(const DebugStream &obj);
  DebugStream& operator=(const DebugStream &obj);
  ~DebugStream();
  //===============================================================
  inline DebugStream &OutEn(bool en)    {out_en=en;    return *this;}
  inline DebugStream &NoNewLine()       {newline=false;return *this;}
  inline DebugStream &Space()           {space=true;   return *this;}
  inline DebugStream &NoSpace()         {space=false;  return *this;}
  //===============================================================
  //=========================================

  //=========================================
  inline DebugStream &print()                                    {return *this;}
  inline DebugStream &operator<<(const DebugStream& stream)      {newline=false;return MayBeSpace();}
  inline DebugStream &operator()()                        {return *this;}
  inline DebugStream &operator<<(void(*)(DebugStreamEndl)) {(*this)<<"\n"; return *this;}

  inline DebugStream &operator<<(void(*)(normal_fg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_NORMAL; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(red_fg_t))    {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_RED; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(green_fg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_GREEN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(yellow_fg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_YELLOW; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(blue_fg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_BLUE; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(magenta_fg_t)){(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_MAGENTA; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(cyan_fg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_CYAN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(white_fg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_WHITE; return (*this).Space();}

  inline DebugStream &operator<<(void(*)(normal_bg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_NORMAL; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(red_bg_t))    {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_RED; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(green_bg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_GREEN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(yellow_bg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_YELLOW; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(blue_bg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_BLUE; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(magenta_bg_t)){(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_MAGENTA; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(cyan_bg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_CYAN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(white_bg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_WHITE; return (*this).Space();}

  inline DebugStream &operator<<(void(*)(debug_general_t))     {(*this).NoSpace()<<normal_fg<<normal_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_status_t))      {(*this).NoSpace()<<red_fg<<cyan_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_warning_t))     {(*this).NoSpace()<<green_fg<<yellow_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_error_t))       {(*this).NoSpace()<<normal_fg<<blue_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_fatal_error_t)) {(*this).NoSpace()<<normal_fg<<red_bg; return (*this).Space();}

  inline DebugStream &operator<<(const char *str)         {(*this)("%s",str);return MayBeSpace();}
  inline DebugStream &operator<<(const std::string& str)  {(*this)("%s",str.c_str());return MayBeSpace();}
  inline DebugStream &operator<<(bool t)                  {(*this)(t?"true":"false");return MayBeSpace();}
  inline DebugStream &operator<<(char t)                  {(*this)("%c",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed short t)          {(*this)("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned short t)        {(*this)("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed int t)            {(*this)("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned int t)          {(*this)("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed long t)           {(*this)("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned long t)         {(*this)("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed long long t)      {(*this)("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned long long t)    {(*this)("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(float t)                 {(*this)("%f",t);return MayBeSpace();}
  inline DebugStream &operator<<(double t)                {(*this)("%lf",t);return MayBeSpace();}
  inline DebugStream &operator<<(const void * t)          {return *this;}
  inline DebugStream &operator<<(std::nullptr_t)          {(*this)("(nullptr)");return *this;}
  inline DebugStream &operator<<(const char16_t* t)       {(*this)("%s",t);return MayBeSpace();}
  inline DebugStream &operator<<(char16_t t)              {(*this)("%c",t);return MayBeSpace();}
  inline DebugStream &operator<<(char32_t t)              {(*this)("%c",t);return MayBeSpace();}

  //======================================
 public:
  DebugStream &operator()(const char *fmt, ...);

 private:
  inline DebugStream &MayBeSpace() {
    if (!newline_&&space) (*this)(" ");
    newline_ = false;
    return *this;
  }

 private:
  std::function<void(const char*,int)> fun{};
  char *DebugStreamBuf;
  int buf_len{256};
  bool out_en{true};
  bool space{true};
  bool newline{true};
  bool newline_{false}; // solve the bug that add newline still add space
};

inline DebugStream::DebugStream
(std::function<void(const char*,int)>  fun_ , int buf_len_ ) : fun(std::move(fun_)) ,buf_len(buf_len_)  {
  DebugStreamBuf = new char[buf_len];
}

inline DebugStream::DebugStream(const DebugStream &obj) {
  this->buf_len = obj.buf_len;
  this->out_en = obj.out_en;
  this->fun = obj.fun;
  // Re-divide a piece of memory, and the function pointer does not need to be reset
  this->DebugStreamBuf = new char[this->buf_len];
}

inline DebugStream &DebugStream::operator=(const DebugStream &obj) {
  if (this->DebugStreamBuf != nullptr) {
    // Compared with the copy construction, the memory originally pointed to by delete is required
    delete DebugStreamBuf;
  }
  this->buf_len = obj.buf_len;
  this->out_en = obj.out_en;
  this->fun = obj.fun;
  // Re-divide a piece of memory, and the function pointer does not need to be reset
  this->DebugStreamBuf = new char[this->buf_len];
  return *this;
}

inline DebugStream::~DebugStream() {
  if(newline) (*this)("\n");
  if (this->DebugStreamBuf != nullptr) {
    delete DebugStreamBuf;
  }
}

inline DebugStream &DebugStream::operator()(const char *fmt, ...) {
#ifdef NO_DEBUG_OUTPUT //
  return *this;
#endif // NO_DEBUG_OUTPUT
  if (!this->out_en) {
    return *this;
  }
  va_list ap;
  va_start(ap, fmt);
  vsprintf((char *) DebugStreamBuf, fmt, ap);
  va_end(ap);
  uint16_t i = strlen((const char *) DebugStreamBuf);
  fun(DebugStreamBuf, i);

  // solve the bug that add newline still add space
  if(DebugStreamBuf[i-1]==0X0A||DebugStreamBuf[i-1]==0X0D) {
    newline_= true;
  }

  return *this;
}


// #define gDebug DebugStream(DebugSendStringCallBack_Default_).print()
// #define gDebug DebugStream(DebugSendStringCallBack_Default_).print()<<normal_fg<<normal_bg
// #define gDebug                                                                 \
//   DebugStream(DebugSendStringCallBack_Default_).print()                        \
//       << WARNING << "GXT_KT:" << STATUS << std::to_string(ros::Time::now().toNSec()/100000000) << "" << GENERAL
#define gDebug                                                                 \
  (DebugStream(DebugSendStringCallBack_Default_).print()                        \
      << STATUS << "GXT_KT:" << GENERAL)

#endif
