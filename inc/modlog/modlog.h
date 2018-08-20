/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_MODLOG_
#define H_MODLOG_

#include <stdio.h>
#include "log/log.h"

#define LOG_COLOR_DEFAULT           "\x1B[0m"
#define LOG_COLOR_BLACK             "\x1B[1;30m"
#define LOG_COLOR_RED               "\x1B[1;31m"
#define LOG_COLOR_GREEN             "\x1B[1;32m"
#define LOG_COLOR_YELLOW            "\x1B[1;33m"
#define LOG_COLOR_BLUE              "\x1B[1;34m"
#define LOG_COLOR_MAGENTA           "\x1B[1;35m"
#define LOG_COLOR_CYAN              "\x1B[1;36m"
#define LOG_COLOR_WHITE             "\x1B[1;37m"

#define MODLOG_DEBUG(ml_mod_, ml_msg_, ...) \
    printf((LOG_COLOR_DEFAULT ml_msg_), ##__VA_ARGS__)

#define MODLOG_INFO(ml_mod_, ml_msg_, ...) \
    printf((LOG_COLOR_GREEN ml_msg_), ##__VA_ARGS__)

#define MODLOG_WARN(ml_mod_, ml_msg_, ...) \
    printf((LOG_COLOR_MAGENTA ml_msg_), ##__VA_ARGS__)

#define MODLOG_ERROR(ml_mod_, ml_msg_, ...) \
    printf((LOG_COLOR_RED ml_msg_), ##__VA_ARGS__)

#define MODLOG_CRITICAL(ml_mod_, ml_msg_, ...) \
    printf((LOG_COLOR_YELLOW ml_msg_), ##__VA_ARGS__)

#define MODLOG(ml_lvl_, ml_mod_, ...) \
    MODLOG_ ## ml_lvl_((ml_mod_), __VA_ARGS__)

#define MODLOG_DFLT(ml_lvl_, ...) \
    MODLOG(ml_lvl_, LOG_MODULE_DEFAULT, __VA_ARGS__)

#endif
