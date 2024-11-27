#enable float point operation, the code below don't work
#ccflags-y += -mfloat-abi=softfp

EXTRA_CFLAGS += -Wno-error=date-time
EXTRA_CFLAGS += -Wno-date-time

lt9211c-objs := lt9211c_module.o lt9211c_log.o Driver/DrvDcsCmd.o Driver/DrvSystem.o Driver/DrvCsc.o Driver/DrvLvdsRx.o Driver/DrvLvdsTx.o Driver/DrvMipiLs.o Driver/DrvMipiRpt.o Driver/DrvMipiRx.o Driver/DrvMipiTx.o Driver/DrvTtlRx.o Driver/DrvTtlTx.o Module/ModSystem.o Module/ModLvdsRx.o Module/ModLvdsTx.o Module/ModMipiLs.o Module/ModMipiRpt.o Module/ModMipiRx.o Module/ModMipiTx.o Module/ModPattern.o Module/ModTtlRx.o Module/ModTtlTx.o lt9211c_mode.o

obj-y        += lt9211c.o

