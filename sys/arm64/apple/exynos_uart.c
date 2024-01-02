/*
 * Copyright (c) 2003 Marcel Moolenaar
 * Copyright (c) 2007-2009 Andrew Turner
 * Copyright (c) 2013 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/cons.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>

#include <arm64/apple/exynos_uart.h>

#include "uart_if.h"

struct exynos_uart_cfg;

#define	DEF_CLK		100000000

static int sscomspeed(long, long);
static int exynos4210_uart_param(struct uart_bas *, int, int, int, int);

/*
 * Low-level UART interface.
 */
static int exynos4210_probe(struct uart_bas *bas);
static void exynos4210_init_common(struct exynos_uart_cfg *cfg,
    struct uart_bas *bas, int, int, int, int);
static void exynos4210_init(struct uart_bas *bas, int, int, int, int);
static void exynos4210_s5l_init(struct uart_bas *bas, int, int, int, int);
static void exynos4210_term(struct uart_bas *bas);
static void exynos4210_putc(struct uart_bas *bas, int);
static int exynos4210_rxready(struct uart_bas *bas);
static int exynos4210_getc(struct uart_bas *bas, struct mtx *mtx);

extern SLIST_HEAD(uart_devinfo_list, uart_devinfo) uart_sysdevs;

enum exynos_uart_type {
	EXUART_4210,
	EXUART_S5L,
};

struct exynos_uart_cfg {
	enum exynos_uart_type	cfg_type;
	uint64_t		cfg_uart_full_mask;
};

static struct exynos_uart_cfg ex4210_cfg = {
	.cfg_type = EXUART_4210,
	.cfg_uart_full_mask = UFSTAT_TXFULL,
};

static struct exynos_uart_cfg s5l_cfg = {
	.cfg_type = EXUART_S5L,
	.cfg_uart_full_mask = UFSTAT_S5L_TXFULL,
};

/* Used purely for cfg mapping; see compat_data later for new devices. */
static struct ofw_compat_data compat_cfg_data[] = {
	{"apple,s5l-uart",	(uintptr_t)&s5l_cfg},
	{NULL,			(uintptr_t)&ex4210_cfg},
};

static int
sscomspeed(long speed, long frequency)
{
	int x;

	if (speed <= 0 || frequency <= 0)
		return (-1);
	x = (frequency / 16) / speed;
	return (x-1);
}

static int
exynos4210_uart_param(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{
	int brd, ulcon;

	ulcon = 0;

	switch(databits) {
	case 5:
		ulcon |= ULCON_LENGTH_5;
		break;
	case 6:
		ulcon |= ULCON_LENGTH_6;
		break;
	case 7:
		ulcon |= ULCON_LENGTH_7;
		break;
	case 8:
		ulcon |= ULCON_LENGTH_8;
		break;
	default:
		return (EINVAL);
	}

	switch (parity) {
	case UART_PARITY_NONE:
		ulcon |= ULCON_PARITY_NONE;
		break;
	case UART_PARITY_ODD:
		ulcon |= ULCON_PARITY_ODD;
		break;
	case UART_PARITY_EVEN:
		ulcon |= ULCON_PARITY_EVEN;
		break;
	case UART_PARITY_MARK:
	case UART_PARITY_SPACE:
	default:
		return (EINVAL);
	}

	if (stopbits == 2)
		ulcon |= ULCON_STOP;

	uart_setreg(bas, SSCOM_ULCON, ulcon);

	/* baudrate may be negative, in which case we just leave it alone. */
	if (baudrate > 0) {
		brd = sscomspeed(baudrate, bas->rclk);
		uart_setreg(bas, SSCOM_UBRDIV, brd);
	}

	return (0);
}

static struct uart_ops uart_exynos4210_ops = {
	.probe = exynos4210_probe,
	.init = exynos4210_init,
	.term = exynos4210_term,
	.putc = exynos4210_putc,
	.rxready = exynos4210_rxready,
	.getc = exynos4210_getc,
};

static struct uart_ops uart_s5l_ops = {
	.probe = exynos4210_probe,
	.init = exynos4210_s5l_init,
	.term = exynos4210_term,
	.putc = exynos4210_putc,
	.rxready = exynos4210_rxready,
	.getc = exynos4210_getc,
};

static int
exynos4210_probe(struct uart_bas *bas)
{

	return (0);
}

static void
exynos4210_init_common(struct exynos_uart_cfg *cfg, struct uart_bas *bas,
    int baudrate, int databits, int stopbits, int parity)
{

	if (bas->rclk == 0)
		bas->rclk = DEF_CLK;

	KASSERT(bas->rclk != 0, ("exynos4210_init: Invalid rclk"));

	bas->driver1 = cfg;

	if (cfg->cfg_type == EXUART_S5L) {
		uart_setreg(bas, SSCOM_UTRSTAT, 0);
	} else {
		uart_setreg(bas, SSCOM_UCON, 0);
	}

	uart_setreg(bas, SSCOM_UFCON,
	    UFCON_TXTRIGGER_8 | UFCON_RXTRIGGER_8 |
	    UFCON_TXFIFO_RESET | UFCON_RXFIFO_RESET |
	    UFCON_FIFO_ENABLE);
	exynos4210_uart_param(bas, baudrate, databits, stopbits, parity);

	/* Enable UART. */
	if (cfg->cfg_type == EXUART_S5L) {
		uart_setreg(bas, SSCOM_UCON, uart_getreg(bas, SSCOM_UCON) |
		    UCON_TOINT | UCON_S5L_RXTHRESH | UCON_S5L_RX_TIMEOUT |
		    UCON_S5L_TXTHRESH);
	} else {
		uart_setreg(bas, SSCOM_UCON, uart_getreg(bas, SSCOM_UCON) |
		    UCON_TXMODE_INT | UCON_RXMODE_INT | UCON_TOINT);
		uart_setreg(bas, SSCOM_UMCON, UMCON_RTS);
	}
}

static void
exynos4210_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{

	return (exynos4210_init_common(&ex4210_cfg, bas, baudrate, databits,
	    stopbits, parity));
}

static void
exynos4210_s5l_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{

	return (exynos4210_init_common(&s5l_cfg, bas, baudrate, databits,
	    stopbits, parity));
}

static void
exynos4210_term(struct uart_bas *bas)
{
	/* XXX */
}

static void
exynos4210_putc(struct uart_bas *bas, int c)
{
	struct exynos_uart_cfg *cfg;

	cfg = bas->driver1;

	while ((bus_space_read_4(bas->bst, bas->bsh, SSCOM_UFSTAT) &
	    cfg->cfg_uart_full_mask) != 0)
		continue;

	uart_setreg(bas, SSCOM_UTXH, c);
}

static int
exynos4210_rxready(struct uart_bas *bas)
{
	struct exynos_uart_cfg *cfg;
	int ufstat, utrstat;

	cfg = bas->driver1;
	utrstat = bus_space_read_4(bas->bst, bas->bsh, SSCOM_UTRSTAT);

	if (cfg->cfg_type == EXUART_S5L) {
		ufstat = bus_space_read_4(bas->bst, bas->bsh, SSCOM_UFSTAT);

		return ((utrstat & UTRSTAT_RXREADY) != 0 ||
		    (ufstat & (UFSTAT_RXCOUNT | UFSTAT_RXFULL)) != 0);
	} else {
		return ((uart_getreg(bas, SSCOM_UTRSTAT) & UTRSTAT_RXREADY) ==
		    UTRSTAT_RXREADY);
	}
}

static int
exynos4210_getc(struct uart_bas *bas, struct mtx *mtx)
{

	while (!exynos4210_rxready(bas)) {
		continue;
	}

	return (uart_getreg(bas, SSCOM_URXH));
}

static int exynos4210_bus_probe(struct uart_softc *sc);
static int exynos4210_bus_attach(struct uart_softc *sc);
static int exynos4210_bus_flush(struct uart_softc *, int);
static int exynos4210_bus_getsig(struct uart_softc *);
static int exynos4210_bus_ioctl(struct uart_softc *, int, intptr_t);
static int exynos4210_bus_ipend(struct uart_softc *);
static int s5l_bus_ipend(struct uart_softc *);
static int exynos4210_bus_param(struct uart_softc *, int, int, int, int);
static int exynos4210_bus_receive(struct uart_softc *);
static int exynos4210_bus_setsig(struct uart_softc *, int);
static int exynos4210_bus_transmit(struct uart_softc *);

static kobj_method_t exynos4210_methods[] = {
	KOBJMETHOD(uart_probe,		exynos4210_bus_probe),
	KOBJMETHOD(uart_attach, 	exynos4210_bus_attach),
	KOBJMETHOD(uart_flush,		exynos4210_bus_flush),
	KOBJMETHOD(uart_getsig,		exynos4210_bus_getsig),
	KOBJMETHOD(uart_ioctl,		exynos4210_bus_ioctl),
	KOBJMETHOD(uart_ipend,		exynos4210_bus_ipend),
	KOBJMETHOD(uart_param,		exynos4210_bus_param),
	KOBJMETHOD(uart_receive,	exynos4210_bus_receive),
	KOBJMETHOD(uart_setsig,		exynos4210_bus_setsig),
	KOBJMETHOD(uart_transmit,	exynos4210_bus_transmit),
	{0, 0 }
};

static kobj_method_t s5l_methods[] = {
	KOBJMETHOD(uart_probe,		exynos4210_bus_probe),
	KOBJMETHOD(uart_attach, 	exynos4210_bus_attach),
	KOBJMETHOD(uart_flush,		exynos4210_bus_flush),
	KOBJMETHOD(uart_getsig,		exynos4210_bus_getsig),
	KOBJMETHOD(uart_ioctl,		exynos4210_bus_ioctl),
	KOBJMETHOD(uart_ipend,		s5l_bus_ipend),
	KOBJMETHOD(uart_param,		exynos4210_bus_param),
	KOBJMETHOD(uart_receive,	exynos4210_bus_receive),
	KOBJMETHOD(uart_setsig,		exynos4210_bus_setsig),
	KOBJMETHOD(uart_transmit,	exynos4210_bus_transmit),
	{0, 0 }
};

int
exynos4210_bus_probe(struct uart_softc *sc)
{

	sc->sc_txfifosz = 16;
	sc->sc_rxfifosz = 16;

	return (0);
}

static int
exynos4210_bus_attach(struct uart_softc *sc)
{
	struct exynos_uart_cfg *cfg;

	sc->sc_hwiflow = 0;
	sc->sc_hwoflow = 0;

	cfg = (struct exynos_uart_cfg *)ofw_bus_search_compatible(sc->sc_dev, compat_cfg_data)->ocd_data;
	MPASS(cfg != NULL);
	MPASS(sc->sc_sysdev == NULL || cfg == sc->sc_sysdev->bas.driver1);
	sc->sc_bas.driver1 = cfg;

	return (0);
}

static int
exynos4210_bus_transmit(struct uart_softc *sc)
{
	struct exynos_uart_cfg *cfg;
	int i;
	int reg;

	cfg = sc->sc_bas.driver1;
	uart_lock(sc->sc_hwmtx);

	for (i = 0; i < sc->sc_txdatasz; i++) {
		exynos4210_putc(&sc->sc_bas, sc->sc_txbuf[i]);
		uart_barrier(&sc->sc_bas);
	}

	if ((bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh, SSCOM_UFSTAT) &
	    cfg->cfg_uart_full_mask) != 0)
		sc->sc_txbusy = 1;

	/* unmask TX interrupt */
	if (cfg->cfg_type == EXUART_S5L) {
		reg = bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh,
		    SSCOM_UCON);
		if ((reg & UCON_S5L_TXTHRESH) == 0) {
			reg |= UCON_S5L_TXTHRESH;
			bus_space_write_4(sc->sc_bas.bst, sc->sc_bas.bsh,
			    UCON_S5L_TXTHRESH, reg);
		}
	} else {
		reg = bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh,
		    SSCOM_UINTM);
		reg &= ~(1 << 2);
		bus_space_write_4(sc->sc_bas.bst, sc->sc_bas.bsh, SSCOM_UINTM,
		    reg);
	}

	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
exynos4210_bus_setsig(struct uart_softc *sc, int sig)
{

	return (0);
}

static int
exynos4210_bus_receive(struct uart_softc *sc)
{
	struct uart_bas *bas;

	bas = &sc->sc_bas;
	while (bus_space_read_4(bas->bst, bas->bsh,
		SSCOM_UFSTAT) & UFSTAT_RXCOUNT)
		uart_rx_put(sc, uart_getreg(&sc->sc_bas, SSCOM_URXH));

	return (0);
}

static int
exynos4210_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	int error;

	if (sc->sc_bas.rclk == 0)
		sc->sc_bas.rclk = DEF_CLK;

	KASSERT(sc->sc_bas.rclk != 0, ("exynos4210_init: Invalid rclk"));

	uart_lock(sc->sc_hwmtx);
	error = exynos4210_uart_param(&sc->sc_bas, baudrate, databits, stopbits,
	    parity);
	uart_unlock(sc->sc_hwmtx);

	return (error);
}

static int
s5l_bus_ipend(struct uart_softc *sc)
{
	int ipend;
	uint32_t uerstat, utrstat;

	ipend = 0;
	uart_lock(sc->sc_hwmtx);
	utrstat = bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh,
	    SSCOM_UTRSTAT);

        if (utrstat & (UTRSTAT_S5L_RXTHRESH | UTRSTAT_S5L_RX_TIMEOUT))
		ipend |= SER_INT_RXREADY;

        if (utrstat & UTRSTAT_S5L_TXTHRESH)
		ipend |= SER_INT_TXIDLE;

	uerstat = bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh,
	    SSCOM_UERSTAT);
	if ((uerstat & UERSTAT_BREAK) != 0)
		ipend |= SER_INT_BREAK;

	bus_space_write_4(sc->sc_bas.bst, sc->sc_bas.bsh, SSCOM_UTRSTAT,
	    utrstat);
	uart_unlock(sc->sc_hwmtx);

	return (ipend);
}

static int
exynos4210_bus_ipend(struct uart_softc *sc)
{
	uint32_t ints;
	uint32_t txempty, rxready;
	int reg;
	int ipend;

	uart_lock(sc->sc_hwmtx);
	ints = bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh, SSCOM_UINTP);
	bus_space_write_4(sc->sc_bas.bst, sc->sc_bas.bsh, SSCOM_UINTP, ints);

	txempty = (1 << 2);
	rxready = (1 << 0);

	ipend = 0;
	if ((ints & txempty) > 0) {
		if (sc->sc_txbusy != 0)
			ipend |= SER_INT_TXIDLE;

		/* mask TX interrupt */
		reg = bus_space_read_4(sc->sc_bas.bst, sc->sc_bas.bsh,
		    SSCOM_UINTM);
		reg |= (1 << 2);
		bus_space_write_4(sc->sc_bas.bst, sc->sc_bas.bsh,
		    SSCOM_UINTM, reg);
	}

	if ((ints & rxready) > 0) {
		ipend |= SER_INT_RXREADY;
	}

	uart_unlock(sc->sc_hwmtx);
	return (ipend);
}

static int
exynos4210_bus_flush(struct uart_softc *sc, int what)
{

	return (0);
}

static int
exynos4210_bus_getsig(struct uart_softc *sc)
{

	return (0);
}

static int
exynos4210_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{

	return (EINVAL);
}

static struct uart_class uart_exynos4210_class = {
	"exynos4210 class",
	exynos4210_methods,
	1,
	.uc_ops = &uart_exynos4210_ops,
	.uc_range = 8,
	.uc_rclk = 0,
	.uc_rshift = 0
};

static struct uart_class uart_s5l_class = {
	"s5l class",
	s5l_methods,
	1,
	.uc_ops = &uart_s5l_ops,
	.uc_range = 8,
	.uc_rclk = 0,
	.uc_rshift = 0
};

static struct ofw_compat_data compat_data[] = {
	{"apple,s5l-uart",		(uintptr_t)&uart_s5l_class},
	{"samsung,exynos4210-uart",	(uintptr_t)&uart_exynos4210_class},
	{NULL,			(uintptr_t)NULL},
};
UART_FDT_CLASS_AND_DEVICE(compat_data);
