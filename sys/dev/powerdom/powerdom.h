/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2016 Michal Meloun <mmel@FreeBSD.org>
 * All rights reserved.
 *
 * Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef DEV_EXTRES_POWERDOM_POWERDOM_H
#define DEV_EXTRES_POWERDOM_POWERDOM_H

#include "opt_platform.h"
#include <sys/types.h>
#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#endif

typedef struct powerdom *powerdom_t;

/*
 * Provider interface
 */
#ifdef FDT
void powerdom_register_ofw_provider(device_t provider_dev);
void powerdom_unregister_ofw_provider(device_t provider_dev);
#endif

/*
 * Consumer interface
 */
int powerdom_get_by_id(device_t consumer_dev, device_t provider_dev,
    intptr_t id, powerdom_t *pd);
void powerdom_release(powerdom_t pd);

int powerdom_enable(powerdom_t pd);
int powerdom_disable(powerdom_t pd);
int powerdom_is_enabled(powerdom_t pd, bool *value);

#ifdef FDT
int powerdom_get_by_ofw_name(device_t consumer_dev, phandle_t node, char *name,
    powerdom_t *pd);
int powerdom_get_by_ofw_idx(device_t consumer_dev, phandle_t node, int idx,
    powerdom_t *pd);
int powerdom_enable_all(device_t consumer_dev);
#endif

#endif /* DEV_EXTRES_POWERDOM_POWERDOM_H */
