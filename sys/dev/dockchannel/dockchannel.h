struct dockchannel_softc;

bool dockchannel_is_masked(struct dockchannel_softc *sc, uint32_t irq);
void dockchannel_mask_irq(struct dockchannel_softc *sc, uint32_t irq);
void dockchannel_unmask_irq(struct dockchannel_softc *sc, uint32_t irq);

// TODO: derive the 2 and 3 from the IRQ struct resources

static bool
dockchannel_is_rx_masked(device_t dev)
{
	device_t parent = device_get_parent(dev);
	struct dockchannel_softc *dcsc = device_get_softc(parent);
	return (dockchannel_is_masked(dcsc, 3));
}

static void
dockchannel_mask_rx(device_t dev)
{
	device_t parent = device_get_parent(dev);
	struct dockchannel_softc *dcsc = device_get_softc(parent);
	dockchannel_mask_irq(dcsc, 3);
}

static void
dockchannel_unmask_rx(device_t dev)
{
	device_t parent = device_get_parent(dev);
	struct dockchannel_softc *dcsc = device_get_softc(parent);
	dockchannel_unmask_irq(dcsc, 3);
}

static void
dockchannel_mask_tx(device_t dev)
{
	device_t parent = device_get_parent(dev);
	struct dockchannel_softc *dcsc = device_get_softc(parent);
	dockchannel_mask_irq(dcsc, 2);
}

static void
dockchannel_unmask_tx(device_t dev)
{
	device_t parent = device_get_parent(dev);
	struct dockchannel_softc *dcsc = device_get_softc(parent);
	dockchannel_unmask_irq(dcsc, 2);
}
