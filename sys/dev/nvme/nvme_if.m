# Copyright (c) 2025 Netlix, Inc
#
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

# Interface from the NVME controller to its children to notify it of certain
# interesting events.

INTERFACE nvme;

HEADER {
	#include "dev/nvme/nvme_private.h"
};

#
# A new namespace is now available
#
METHOD int ns_added {
	device_t	dev;		/* nvme device */
	struct nvme_namespace *ns;	/* information about the namespace */
};

#
# A namespace has been removed
#
METHOD int ns_removed {
	device_t	dev;		/* nvme device */
	struct nvme_namespace *ns;	/* information about the namespace */
};

#
# A namespace has been changed somehow
#
METHOD int ns_changed {
	device_t	dev;		/* nvme device */
	uint32_t	nsid;		/* nsid that just changed */
};

#
# The controller has failed
#
METHOD int controller_failed {
	device_t	dev;		/* nvme device */
};

#
# Async completion
#
METHOD int handle_aen {
	device_t	dev;		/* nvme device */
	const struct nvme_completion *cpl; /* Completion for this async event */
	uint32_t	pg_nr;		/* Page number reported by async event */
	void		*page;		/* Contents of the page */
	uint32_t	page_len;	/* Length of the page */
};

METHOD int delayed_attach {
	device_t dev;
	struct nvme_controller *ctrlr;
};

METHOD void enable {
	device_t dev;
};

METHOD uint32_t sq_enter {
	device_t dev;
	struct nvme_qpair *qpair;
	struct nvme_tracker *tr;
} DEFAULT nvme_qpair_sq_enter;

METHOD void sq_leave {
	device_t dev;
	struct nvme_qpair *qpair;
	struct nvme_tracker *tr;
} DEFAULT nvme_qpair_sq_leave;

METHOD void cq_done {
	device_t dev;
	struct nvme_qpair *qpair;
	struct nvme_tracker *tr;
} DEFAULT nvme_qpair_cq_done;

METHOD int qpair_construct {
	device_t dev;
	struct nvme_qpair *qpair;
    uint32_t num_entries;
	uint32_t num_trackers;
    struct nvme_controller *ctrlr;
} DEFAULT nvme_qpair_construct;
