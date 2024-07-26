#-
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# $FreeBSD$
#

#include <dev/nvme/nvme_private.h>

INTERFACE nvme;

CODE {
	static void
	null_nvme_enable(struct nvme_controller *ctrlr)
	{

	}
};

METHOD void enable {
	device_t dev;
	struct nvme_controller *ctrlr;
} DEFAULT null_nvme_enable;

METHOD uint32_t sq_enter {
	device_t dev;
	struct nvme_controller *ctrlr;
	struct nvme_qpair *qpair;
} DEFAULT nvme_qpair_sq_enter;

METHOD void sq_leave {
	device_t dev;
	struct nvme_controller *ctrlr;
	struct nvme_qpair *qpair;
} DEFAULT nvme_qpair_sq_leave;
