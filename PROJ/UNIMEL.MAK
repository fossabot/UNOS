#
# Borland C++ IDE generated makefile
#
.AUTODEPEND


#
# Borland C++ tools
#
IMPLIB  = Implib
BCCDOS  = Bcc +BccDos.cfg 
TLINK   = TLink
TLIB    = TLib
TASM    = Tasm
#
# IDE macros
#


#
# Options
#
IDE_LFLAGSDOS =  -L\bc4\lib
IDE_BFLAGS = 
LLATDOS_unimeldexe =  -LC:\BORLANDC\LIB;H:\BC4\LIB;C:\TASM; -c -Tde
RLATDOS_unimeldexe =  -IC:\BORLANDC\INCLUDE;..\GLOBALH;H:\BC4\INCLUDE;C:\TASM;
BLATDOS_unimeldexe = 
LEAT_unimeldexe = $(LLATDOS_unimeldexe)
REAT_unimeldexe = $(RLATDOS_unimeldexe)
BEAT_unimeldexe = $(BLATDOS_unimeldexe)

#
# Dependency List
#
Dep_unimel = \
   C:\UNIMEL\PROJ\unimel.exe

unimel : BccDos.cfg $(Dep_unimel)
  echo MakeNode unimel

Dep_CcbUNIMELbPROJbunimeldexe = \
   C:\UNIMEL\OBJ\plcscr.obj\
   C:\UNIMEL\OBJ\screen.obj\
   C:\UNIMEL\OBJ\tasknull.obj\
   C:\UNIMEL\OBJ\iohw.obj\
   C:\UNIMEL\OBJ\rclock.obj\
   C:\UNIMEL\OBJ\ioscr.obj\
   C:\UNIMEL\OBJ\slp.obj\
   C:\UNIMEL\OBJ\queue.obj\
   C:\UNIMEL\OBJ\decoder.obj\
   C:\UNIMEL\OBJ\comprot.obj\
   C:\UNIMEL\OBJ\plc.obj\
   C:\UNIMEL\OBJ\cmclock.obj\
   C:\UNIMEL\OBJ\error.obj\
   C:\UNIMEL\OBJ\atcumain.obj\
   C:\UNIMEL\OBJ\nvrampc.obj\
   C:\UNIMEL\OBJ\poscon.obj\
   C:\UNIMEL\OBJ\protocol.obj\
   C:\UNIMEL\OBJ\seq.obj\
   C:\UNIMEL\OBJ\seqscr.obj\
   C:\UNIMEL\OBJ\kbseq.obj\
   C:\UNIMEL\OBJ\wdog.obj\
   C:\UNIMEL\OBJ\startrak.obj\
   C:\UNIMEL\OBJ\tunekbd.obj\
   C:\UNIMEL\OBJ\tparser.obj\
   C:\UNIMEL\OBJ\tuneout.obj\
   C:\UNIMEL\OBJ\tunescr.obj\
   C:\UNIMEL\OBJ\serial.obj\
   C:\UNIMEL\OBJ\serinit.obj\
   C:\UNIMEL\OBJ\serint.obj\
   C:\UNIMEL\OBJ\comdec.obj\
   C:\UNIMEL\OBJ\kbtask.obj\
   C:\UNIMEL\OBJ\clock.obj\
   C:\UNIMEL\OBJ\fpx.obj\
   C:\UNIMEL\OBJ\hwpc.obj\
   C:\UNIMEL\OBJ\kbddrv.obj\
   C:\UNIMEL\OBJ\pcscr.obj\
   C:\UNIMEL\OBJ\unos.obj\
   C:\UNIMEL\OBJ\unosasm.obj\
   C:\UNIMEL\OBJ\intels3.obj\
   C:\UNIMEL\OBJ\simintel.obj

C:\UNIMEL\PROJ\unimel.exe : $(Dep_CcbUNIMELbPROJbunimeldexe)
  $(TLINK)   @&&|
 /v $(IDE_LFLAGSDOS) $(LEAT_unimeldexe) +
H:\BC4\LIB\c0h.obj+
C:\UNIMEL\OBJ\plcscr.obj+
C:\UNIMEL\OBJ\screen.obj+
C:\UNIMEL\OBJ\tasknull.obj+
C:\UNIMEL\OBJ\iohw.obj+
C:\UNIMEL\OBJ\rclock.obj+
C:\UNIMEL\OBJ\ioscr.obj+
C:\UNIMEL\OBJ\slp.obj+
C:\UNIMEL\OBJ\queue.obj+
C:\UNIMEL\OBJ\decoder.obj+
C:\UNIMEL\OBJ\comprot.obj+
C:\UNIMEL\OBJ\plc.obj+
C:\UNIMEL\OBJ\cmclock.obj+
C:\UNIMEL\OBJ\error.obj+
C:\UNIMEL\OBJ\atcumain.obj+
C:\UNIMEL\OBJ\nvrampc.obj+
C:\UNIMEL\OBJ\poscon.obj+
C:\UNIMEL\OBJ\protocol.obj+
C:\UNIMEL\OBJ\seq.obj+
C:\UNIMEL\OBJ\seqscr.obj+
C:\UNIMEL\OBJ\kbseq.obj+
C:\UNIMEL\OBJ\wdog.obj+
C:\UNIMEL\OBJ\startrak.obj+
C:\UNIMEL\OBJ\tunekbd.obj+
C:\UNIMEL\OBJ\tparser.obj+
C:\UNIMEL\OBJ\tuneout.obj+
C:\UNIMEL\OBJ\tunescr.obj+
C:\UNIMEL\OBJ\serial.obj+
C:\UNIMEL\OBJ\serinit.obj+
C:\UNIMEL\OBJ\serint.obj+
C:\UNIMEL\OBJ\comdec.obj+
C:\UNIMEL\OBJ\kbtask.obj+
C:\UNIMEL\OBJ\clock.obj+
C:\UNIMEL\OBJ\fpx.obj+
C:\UNIMEL\OBJ\hwpc.obj+
C:\UNIMEL\OBJ\kbddrv.obj+
C:\UNIMEL\OBJ\pcscr.obj+
C:\UNIMEL\OBJ\unos.obj+
C:\UNIMEL\OBJ\unosasm.obj+
C:\UNIMEL\OBJ\intels3.obj+
C:\UNIMEL\OBJ\simintel.obj
$<,$*
H:\BC4\LIB\graphics.lib+
H:\BC4\LIB\fp87.lib+
H:\BC4\LIB\mathh.lib+
H:\BC4\LIB\ch.lib

|

C:\UNIMEL\OBJ\plcscr.obj :  ..\iodriver\plcscr.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\iodriver\plcscr.c

C:\UNIMEL\OBJ\screen.obj :  ..\screen\screen.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\screen\screen.c

C:\UNIMEL\OBJ\tasknull.obj :  ..\unos\tasknull.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\tasknull.c

C:\UNIMEL\OBJ\iohw.obj :  ..\iodriver\iohw.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\iodriver\iohw.c

C:\UNIMEL\OBJ\rclock.obj :  ..\unos\rclock.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\rclock.c

C:\UNIMEL\OBJ\ioscr.obj :  ..\xcomms\ioscr.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\xcomms\ioscr.c

C:\UNIMEL\OBJ\slp.obj :  ..\xcomms\slp.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\xcomms\slp.c

C:\UNIMEL\OBJ\queue.obj :  ..\xcomms\queue.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\xcomms\queue.c

C:\UNIMEL\OBJ\decoder.obj :  ..\xcomms\decoder.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\xcomms\decoder.c

C:\UNIMEL\OBJ\comprot.obj :  ..\xcomms\comprot.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\xcomms\comprot.c

C:\UNIMEL\OBJ\plc.obj :  ..\iodriver\plc.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\iodriver\plc.c

C:\UNIMEL\OBJ\cmclock.obj :  ..\clock\cmclock.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\clock\cmclock.c

C:\UNIMEL\OBJ\error.obj :  ..\errhand\error.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\errhand\error.c

C:\UNIMEL\OBJ\atcumain.obj :  ..\main\atcumain.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\main\atcumain.c

C:\UNIMEL\OBJ\nvrampc.obj :  ..\nvram\nvrampc.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\nvram\nvrampc.c

C:\UNIMEL\OBJ\poscon.obj :  ..\posloop\poscon.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\posloop\poscon.c

C:\UNIMEL\OBJ\protocol.obj :  ..\protocol\protocol.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\protocol\protocol.c

C:\UNIMEL\OBJ\seq.obj :  ..\sequ\seq.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\sequ\seq.c

C:\UNIMEL\OBJ\seqscr.obj :  ..\sequ\seqscr.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\sequ\seqscr.c

C:\UNIMEL\OBJ\kbseq.obj :  ..\sequ\kbseq.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\sequ\kbseq.c

C:\UNIMEL\OBJ\wdog.obj :  ..\sequ\wdog.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\sequ\wdog.c

C:\UNIMEL\OBJ\startrak.obj :  ..\startrk\startrak.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\startrk\startrak.c

C:\UNIMEL\OBJ\tunekbd.obj :  ..\tune\tunekbd.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\tune\tunekbd.c

C:\UNIMEL\OBJ\tparser.obj :  ..\tune\tparser.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\tune\tparser.c

C:\UNIMEL\OBJ\tuneout.obj :  ..\tune\tuneout.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\tune\tuneout.c

C:\UNIMEL\OBJ\tunescr.obj :  ..\tune\tunescr.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\tune\tunescr.c

C:\UNIMEL\OBJ\serial.obj :  ..\unos\serial.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\serial.c

C:\UNIMEL\OBJ\serinit.obj :  ..\unos\serinit.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\serinit.c

C:\UNIMEL\OBJ\serint.obj :  ..\unos\serint.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\serint.c

C:\UNIMEL\OBJ\comdec.obj :  ..\unos\comdec.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\comdec.c

C:\UNIMEL\OBJ\kbtask.obj :  ..\unos\kbtask.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\kbtask.c

C:\UNIMEL\OBJ\clock.obj :  ..\unos\clock.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\clock.c

C:\UNIMEL\OBJ\fpx.obj :  ..\unos\fpx.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\fpx.c

C:\UNIMEL\OBJ\hwpc.obj :  ..\unos\hwpc.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\hwpc.c

C:\UNIMEL\OBJ\kbddrv.obj :  ..\unos\kbddrv.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\kbddrv.c

C:\UNIMEL\OBJ\pcscr.obj :  ..\unos\pcscr.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\pcscr.c

C:\UNIMEL\OBJ\unos.obj :  ..\unos\unos.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\unos\unos.c

C:\UNIMEL\OBJ\unosasm.obj :  ..\unos\unosasm.c
  $(TASM) ..\unos\unosasm.c

C:\UNIMEL\OBJ\intels3.obj :  ..\intelsat\intels3.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\intelsat\intels3.c

C:\UNIMEL\OBJ\simintel.obj :  ..\intelsat\simintel.c
  $(BCCDOS) -c $(CEAT_unimeldexe) -o$@ ..\intelsat\simintel.c

# Compiler configuration file
BccDos.cfg : 
   Copy &&|
-W-
-R
-v
-vi
-X-
-H
-I\bc4\include
-H=unimel.csm
-3
-ff-
-X-
-Oe
-Z
-k-
-vi-
-H-
-IC:\BORLANDC\INCLUDE;..\GLOBALH;H:\BC4\INCLUDE;C:\TASM;
-n
-f287
| $@


