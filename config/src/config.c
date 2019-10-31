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

struct conf_handler_head conf_handlers;

static struct ble_npl_mutex conf_mtx;

void
conf_init(void)
{
    ble_npl_mutex_init(&conf_mtx);
    SLIST_INIT(&conf_handlers);
    conf_store_init();
}

void
conf_lock(void)
{
    ble_npl_mutex_pend(&conf_mtx, BLE_NPL_TIME_FOREVER);
}

void
conf_unlock(void)
{
    ble_npl_mutex_release(&conf_mtx);
}

int
conf_register(struct conf_handler *handler)
{
    conf_lock();
    SLIST_INSERT_HEAD(&conf_handlers, handler, ch_list);
    conf_unlock();
    return 0;
}

/*
 * Find conf_handler based on name.
 */
struct conf_handler *
conf_handler_lookup(char *name)
{
    struct conf_handler *ch;

    SLIST_FOREACH(ch, &conf_handlers, ch_list) {
        if (!strcmp(name, ch->ch_name)) {
            return ch;
        }
    }
    return NULL;
}

/*
 * Separate string into argv array.
 */
int
conf_parse_name(char *name, int *name_argc, char *name_argv[])
{
    char *tok;
    char *sep = CONF_NAME_SEPARATOR;
    int i;

    tok = strtok(name, sep);

    i = 0;
    while (tok) {
        name_argv[i++] = tok;
        tok = strtok(NULL, sep);
    }
    *name_argc = i;

    return 0;
}

struct conf_handler *
conf_parse_and_lookup(char *name, int *name_argc, char *name_argv[])
{
    int rc;

    rc = conf_parse_name(name, name_argc, name_argv);
    if (rc) {
        return NULL;
    }
    return conf_handler_lookup(name_argv[0]);
}

int
conf_set_value(char *name, char *val_str)
{
    int name_argc;
    char *name_argv[CONF_MAX_DIR_DEPTH];
    struct conf_handler *ch;
    int rc;

    conf_lock();
    ch = conf_parse_and_lookup(name, &name_argc, name_argv);
    if (!ch) {
        rc = OS_INVALID_PARM;
        goto out;
    }
    rc = ch->ch_set(name_argc - 1, &name_argv[1], val_str);
out:
    conf_unlock();
    return rc;
}

/*
 * Get value in printable string form. If value is not string, the value
 * will be filled in *buf.
 * Return value will be pointer to beginning of that buffer,
 * except for string it will pointer to beginning of string.
 */
char *
conf_get_value(char *name, char *buf, int buf_len)
{
    int name_argc;
    char *name_argv[CONF_MAX_DIR_DEPTH];
    struct conf_handler *ch;
    char *rval = NULL;

    conf_lock();
    ch = conf_parse_and_lookup(name, &name_argc, name_argv);
    if (!ch) {
        goto out;
    }

    if (!ch->ch_get) {
        goto out;
    }
    rval = ch->ch_get(name_argc - 1, &name_argv[1], buf, buf_len);
out:
    conf_unlock();
    return rval;
}

int
conf_commit(char *name)
{
    int name_argc;
    char *name_argv[CONF_MAX_DIR_DEPTH];
    struct conf_handler *ch;
    int rc;
    int rc2;

    conf_lock();
    if (name) {
        ch = conf_parse_and_lookup(name, &name_argc, name_argv);
        if (!ch) {
            rc = OS_INVALID_PARM;
            goto out;
        }
        if (ch->ch_commit) {
            rc = ch->ch_commit();
        } else {
            rc = 0;
        }
    } else {
        rc = 0;
        SLIST_FOREACH(ch, &conf_handlers, ch_list) {
            if (ch->ch_commit) {
                rc2 = ch->ch_commit();
                if (!rc) {
                    rc = rc2;
                }
            }
        }
    }
out:
    conf_unlock();
    return rc;
}
