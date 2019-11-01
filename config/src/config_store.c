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

#include <string.h>
#include <stdio.h>

#include "os/os.h"
#include "config/config.h"
#include "config_priv.h"

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = block_device_read,
    .prog  = block_device_prog,
    .erase = block_device_erase,
    .sync  = block_device_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = FLASH_PAGE_SIZE,
    .block_count = 2,
    .cache_size = 16,
    .lookahead_size = 16,
};
// variables used by the filesystem
static lfs_t lfs;

int
conf_load(void)
{
    /*
     * for every config store
     *    load config
     *    apply config
     *    commit all
     */
    conf_lock();
    conf_set_value(NULL, NULL);
    conf_unlock();
    return conf_commit(NULL);
}

/*
 * Append a single value to persisted config. Don't store duplicate value.
 */
int
conf_save_one(const char *name, char *value)
{
    lfs_file_t file;
    int err;

    if (NULL == name) {
        return OS_INVALID_PARM;
    }

    conf_lock();

    err = lfs_file_open(&lfs, &file, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
    if (err) {
        console_printf("Can't open the file %s (err %d)\n", name, err);
        goto out;
    }

    if (value) {
        err = lfs_file_write(&lfs, &file, value, strlen(value) + 1);
        if (err) {
            console_printf("Can't write the file %s (err %d)\n", name, err);
        }
    } else {
        err = lfs_file_truncate(&lfs, &file, 0);
        if (err) {
            console_printf("Can't truncate the file %s (err %d)\n", name, err);
        }
    }

    // remember the storage is not updated until the file is closed successfully
    err = lfs_file_close(&lfs, &file);
    if (err) {
        console_printf("Can't close the file %s (err %d)\n", name, err);
    }

out:
    conf_unlock();
    return (LFS_ERR_OK == err) ? OS_OK : OS_ERROR;
}

void
conf_store_init(void)
{
    int err;

    conf_lock();
    /* Initilize flash as block device */
    err = block_device_init();
    if (err) {
        console_printf("Can't initilize the flash (err %d)\n", err);
        goto out;
    }
    /* Mount the filesystem */
    err = lfs_mount(&lfs, &cfg);
    if (err) {
        /* this should only happen on the first boot */
        console_printf("Can't mount the filesystem (err %d)\n", err);
        lfs_format(&lfs, &cfg);
        err = lfs_mount(&lfs, &cfg);
        if (err) {
            console_printf("Something is wrong, abort it (err %d)\n", err);
            goto out;
        }
    }

out:
    conf_unlock();
}
