# Test Repository for Concurrent Stepper Driving Code

## Reproductions

### `with_pio` binary

Replacing `PIO0` with `PIO1` stops the code from working, even though both state machines blocks should function identically.

### `multiple-drivers` binary

DRIVER_1 is controlled properly, whereas DRIVER_2 isn't. If the names are switched, the same occurs - indicating that the issue isn't in incorrect pin configuration.
This same issue occurs both when trying to use 2 state machines from the same block, or 2 state machines from different blocks.
It seems to be caused by interrupts not firing properly. When enabling both, only the PIO 0 interrupt fires:
```
pio0.irq1().enable_sm_interrupt(1);
pio1.irq1().enable_sm_interrupt(1);
```

I thought this might've been due to the IRQ relative addressing (using mod 4 sums), but no amount of changing the `irq <num>` instruction caused the other interrupt to fire.
