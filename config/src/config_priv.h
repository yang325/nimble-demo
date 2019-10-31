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

#ifndef __CONFIG_PRIV_H_
#define __CONFIG_PRIV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lfs.h"
#include "stm32f1xx_hal.h"

/*
 * Lock config subsystem.
 */
void conf_lock(void);
void conf_unlock(void);

int block_device_init(void);
int block_device_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size);
int block_device_prog(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size);
int block_device_erase(const struct lfs_config *c, lfs_block_t block);
int block_device_sync(const struct lfs_config *c);

struct conf_handler *conf_parse_and_lookup(char *name, int *name_argc,
                                           char *name_argv[]);


SLIST_HEAD(conf_handler_head, conf_handler);
extern struct conf_handler_head conf_handlers;

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_PRIV_H_ */
