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
IDE_LFLAGSDOS =  -LC:\BC4\LIB
IDE_BFLAGS = 
LLATDOS_ts3000dexe =  -v -LC:\BC4\LIB; -C -c -Tde
RLATDOS_ts3000dexe =  -IC:\BC4\INCLUDE; ..\GLOBALH;
BLATDOS_ts3000dexe = 
LEAT_ts3000dexe = $(LLATDOS_ts3000dexe)
REAT_ts3000dexe = $(RLATDOS_ts3000dexe)
BEAT_ts3000dexe = $(BLATDOS_ts3000dexe)
CLATDOS_ddbunosbserialdc =  -f
LLATDOS_ddbunosbserialdc = 
RLATDOS_ddbunosbserialdc = 
BLATDOS_ddbunosbserialdc = 
CEAT_ddbunosbserialdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbserialdc)
LEAT_ddbunosbserialdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbserialdc)
REAT_ddbunosbserialdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbserialdc)
BEAT_ddbunosbserialdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbserialdc)
CLATDOS_ddbunosbserintdc =  -f
LLATDOS_ddbunosbserintdc = 
RLATDOS_ddbunosbserintdc = 
BLATDOS_ddbunosbserintdc = 
CEAT_ddbunosbserintdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbserintdc)
LEAT_ddbunosbserintdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbserintdc)
REAT_ddbunosbserintdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbserintdc)
BEAT_ddbunosbserintdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbserintdc)
CLATDOS_ddbunosbserinitdc =  -f
LLATDOS_ddbunosbserinitdc = 
RLATDOS_ddbunosbserinitdc = 
BLATDOS_ddbunosbserinitdc = 
CEAT_ddbunosbserinitdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbserinitdc)
LEAT_ddbunosbserinitdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbserinitdc)
REAT_ddbunosbserinitdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbserinitdc)
BEAT_ddbunosbserinitdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbserinitdc)
CLATDOS_ddbtunebtparserdc =  -f
LLATDOS_ddbtunebtparserdc = 
RLATDOS_ddbtunebtparserdc = 
BLATDOS_ddbtunebtparserdc = 
CEAT_ddbtunebtparserdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbtunebtparserdc)
LEAT_ddbtunebtparserdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbtunebtparserdc)
REAT_ddbtunebtparserdc = $(REAT_ts3000dexe) $(RLATDOS_ddbtunebtparserdc)
BEAT_ddbtunebtparserdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbtunebtparserdc)
CLATDOS_ddbtunebtunekbddc =  -f
LLATDOS_ddbtunebtunekbddc = 
RLATDOS_ddbtunebtunekbddc = 
BLATDOS_ddbtunebtunekbddc = 
CEAT_ddbtunebtunekbddc = $(CEAT_ts3000dexe) $(CLATDOS_ddbtunebtunekbddc)
LEAT_ddbtunebtunekbddc = $(LEAT_ts3000dexe) $(LLATDOS_ddbtunebtunekbddc)
REAT_ddbtunebtunekbddc = $(REAT_ts3000dexe) $(RLATDOS_ddbtunebtunekbddc)
BEAT_ddbtunebtunekbddc = $(BEAT_ts3000dexe) $(BLATDOS_ddbtunebtunekbddc)
CLATDOS_ddbtunebtuneoutdc =  -f
LLATDOS_ddbtunebtuneoutdc = 
RLATDOS_ddbtunebtuneoutdc = 
BLATDOS_ddbtunebtuneoutdc = 
CEAT_ddbtunebtuneoutdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbtunebtuneoutdc)
LEAT_ddbtunebtuneoutdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbtunebtuneoutdc)
REAT_ddbtunebtuneoutdc = $(REAT_ts3000dexe) $(RLATDOS_ddbtunebtuneoutdc)
BEAT_ddbtunebtuneoutdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbtunebtuneoutdc)
CLATDOS_ddbtunebtunescrdc =  -f
LLATDOS_ddbtunebtunescrdc = 
RLATDOS_ddbtunebtunescrdc = 
BLATDOS_ddbtunebtunescrdc = 
CEAT_ddbtunebtunescrdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbtunebtunescrdc)
LEAT_ddbtunebtunescrdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbtunebtunescrdc)
REAT_ddbtunebtunescrdc = $(REAT_ts3000dexe) $(RLATDOS_ddbtunebtunescrdc)
BEAT_ddbtunebtunescrdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbtunebtunescrdc)
CLATDOS_ddbunosbcomdecdc =  -f
LLATDOS_ddbunosbcomdecdc = 
RLATDOS_ddbunosbcomdecdc = 
BLATDOS_ddbunosbcomdecdc = 
CEAT_ddbunosbcomdecdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbcomdecdc)
LEAT_ddbunosbcomdecdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbcomdecdc)
REAT_ddbunosbcomdecdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbcomdecdc)
BEAT_ddbunosbcomdecdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbcomdecdc)
CLATDOS_ddbintelsatbintels3dc =  -f
LLATDOS_ddbintelsatbintels3dc = 
RLATDOS_ddbintelsatbintels3dc = 
BLATDOS_ddbintelsatbintels3dc = 
CEAT_ddbintelsatbintels3dc = $(CEAT_ts3000dexe) $(CLATDOS_ddbintelsatbintels3dc)
LEAT_ddbintelsatbintels3dc = $(LEAT_ts3000dexe) $(LLATDOS_ddbintelsatbintels3dc)
REAT_ddbintelsatbintels3dc = $(REAT_ts3000dexe) $(RLATDOS_ddbintelsatbintels3dc)
BEAT_ddbintelsatbintels3dc = $(BEAT_ts3000dexe) $(BLATDOS_ddbintelsatbintels3dc)
CLATDOS_ddbintelsatbsiminteldc =  -f
LLATDOS_ddbintelsatbsiminteldc = 
RLATDOS_ddbintelsatbsiminteldc = 
BLATDOS_ddbintelsatbsiminteldc = 
CEAT_ddbintelsatbsiminteldc = $(CEAT_ts3000dexe) $(CLATDOS_ddbintelsatbsiminteldc)
LEAT_ddbintelsatbsiminteldc = $(LEAT_ts3000dexe) $(LLATDOS_ddbintelsatbsiminteldc)
REAT_ddbintelsatbsiminteldc = $(REAT_ts3000dexe) $(RLATDOS_ddbintelsatbsiminteldc)
BEAT_ddbintelsatbsiminteldc = $(BEAT_ts3000dexe) $(BLATDOS_ddbintelsatbsiminteldc)
CLATDOS_ddbsequbkbseqdc =  -f
LLATDOS_ddbsequbkbseqdc = 
RLATDOS_ddbsequbkbseqdc = 
BLATDOS_ddbsequbkbseqdc = 
CEAT_ddbsequbkbseqdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbsequbkbseqdc)
LEAT_ddbsequbkbseqdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbsequbkbseqdc)
REAT_ddbsequbkbseqdc = $(REAT_ts3000dexe) $(RLATDOS_ddbsequbkbseqdc)
BEAT_ddbsequbkbseqdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbsequbkbseqdc)
CLATDOS_ddbunosbkbtaskdc =  -f
LLATDOS_ddbunosbkbtaskdc = 
RLATDOS_ddbunosbkbtaskdc = 
BLATDOS_ddbunosbkbtaskdc = 
CEAT_ddbunosbkbtaskdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbkbtaskdc)
LEAT_ddbunosbkbtaskdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbkbtaskdc)
REAT_ddbunosbkbtaskdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbkbtaskdc)
BEAT_ddbunosbkbtaskdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbkbtaskdc)
CLATDOS_ddbnvrambnvrampcdc =  -f
LLATDOS_ddbnvrambnvrampcdc = 
RLATDOS_ddbnvrambnvrampcdc = 
BLATDOS_ddbnvrambnvrampcdc = 
CEAT_ddbnvrambnvrampcdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbnvrambnvrampcdc)
LEAT_ddbnvrambnvrampcdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbnvrambnvrampcdc)
REAT_ddbnvrambnvrampcdc = $(REAT_ts3000dexe) $(RLATDOS_ddbnvrambnvrampcdc)
BEAT_ddbnvrambnvrampcdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbnvrambnvrampcdc)
CLATDOS_ddbscreenbscreendc =  -f
LLATDOS_ddbscreenbscreendc = 
RLATDOS_ddbscreenbscreendc = 
BLATDOS_ddbscreenbscreendc = 
CEAT_ddbscreenbscreendc = $(CEAT_ts3000dexe) $(CLATDOS_ddbscreenbscreendc)
LEAT_ddbscreenbscreendc = $(LEAT_ts3000dexe) $(LLATDOS_ddbscreenbscreendc)
REAT_ddbscreenbscreendc = $(REAT_ts3000dexe) $(RLATDOS_ddbscreenbscreendc)
BEAT_ddbscreenbscreendc = $(BEAT_ts3000dexe) $(BLATDOS_ddbscreenbscreendc)
CLATDOS_ddbunosbclockdc =  -f
LLATDOS_ddbunosbclockdc = 
RLATDOS_ddbunosbclockdc = 
BLATDOS_ddbunosbclockdc = 
CEAT_ddbunosbclockdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbclockdc)
LEAT_ddbunosbclockdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbclockdc)
REAT_ddbunosbclockdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbclockdc)
BEAT_ddbunosbclockdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbclockdc)
CLATDOS_ddbunosbhwpcdc =  -f -ff-
LLATDOS_ddbunosbhwpcdc = 
RLATDOS_ddbunosbhwpcdc = 
BLATDOS_ddbunosbhwpcdc = 
CEAT_ddbunosbhwpcdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbhwpcdc)
LEAT_ddbunosbhwpcdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbhwpcdc)
REAT_ddbunosbhwpcdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbhwpcdc)
BEAT_ddbunosbhwpcdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbhwpcdc)
CLATDOS_ddbunosbkbddrvdc =  -f
LLATDOS_ddbunosbkbddrvdc = 
RLATDOS_ddbunosbkbddrvdc = 
BLATDOS_ddbunosbkbddrvdc = 
CEAT_ddbunosbkbddrvdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbkbddrvdc)
LEAT_ddbunosbkbddrvdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbkbddrvdc)
REAT_ddbunosbkbddrvdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbkbddrvdc)
BEAT_ddbunosbkbddrvdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbkbddrvdc)
CLATDOS_ddbunosbtasknulldc =  -f
LLATDOS_ddbunosbtasknulldc = 
RLATDOS_ddbunosbtasknulldc = 
BLATDOS_ddbunosbtasknulldc = 
CEAT_ddbunosbtasknulldc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbtasknulldc)
LEAT_ddbunosbtasknulldc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbtasknulldc)
REAT_ddbunosbtasknulldc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbtasknulldc)
BEAT_ddbunosbtasknulldc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbtasknulldc)
CLATDOS_ddbunosbpcscrdc =  -f
LLATDOS_ddbunosbpcscrdc = 
RLATDOS_ddbunosbpcscrdc = 
BLATDOS_ddbunosbpcscrdc = 
CEAT_ddbunosbpcscrdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbpcscrdc)
LEAT_ddbunosbpcscrdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbpcscrdc)
REAT_ddbunosbpcscrdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbpcscrdc)
BEAT_ddbunosbpcscrdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbpcscrdc)
CLATDOS_ddbunosbunosdc =  -f
LLATDOS_ddbunosbunosdc = 
RLATDOS_ddbunosbunosdc = 
BLATDOS_ddbunosbunosdc = 
CEAT_ddbunosbunosdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbunosbunosdc)
LEAT_ddbunosbunosdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbunosbunosdc)
REAT_ddbunosbunosdc = $(REAT_ts3000dexe) $(RLATDOS_ddbunosbunosdc)
BEAT_ddbunosbunosdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbunosbunosdc)
CLATDOS_ddbclockbcmclockdc =  -f
LLATDOS_ddbclockbcmclockdc = 
RLATDOS_ddbclockbcmclockdc = 
BLATDOS_ddbclockbcmclockdc = 
CEAT_ddbclockbcmclockdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbclockbcmclockdc)
LEAT_ddbclockbcmclockdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbclockbcmclockdc)
REAT_ddbclockbcmclockdc = $(REAT_ts3000dexe) $(RLATDOS_ddbclockbcmclockdc)
BEAT_ddbclockbcmclockdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbclockbcmclockdc)
CLATDOS_ddberrhandberrordc =  -f
LLATDOS_ddberrhandberrordc = 
RLATDOS_ddberrhandberrordc = 
BLATDOS_ddberrhandberrordc = 
CEAT_ddberrhandberrordc = $(CEAT_ts3000dexe) $(CLATDOS_ddberrhandberrordc)
LEAT_ddberrhandberrordc = $(LEAT_ts3000dexe) $(LLATDOS_ddberrhandberrordc)
REAT_ddberrhandberrordc = $(REAT_ts3000dexe) $(RLATDOS_ddberrhandberrordc)
BEAT_ddberrhandberrordc = $(BEAT_ts3000dexe) $(BLATDOS_ddberrhandberrordc)
CLATDOS_ddbiodriverbsimuliofdc =  -f
LLATDOS_ddbiodriverbsimuliofdc = 
RLATDOS_ddbiodriverbsimuliofdc = 
BLATDOS_ddbiodriverbsimuliofdc = 
CEAT_ddbiodriverbsimuliofdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbiodriverbsimuliofdc)
LEAT_ddbiodriverbsimuliofdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbiodriverbsimuliofdc)
REAT_ddbiodriverbsimuliofdc = $(REAT_ts3000dexe) $(RLATDOS_ddbiodriverbsimuliofdc)
BEAT_ddbiodriverbsimuliofdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbiodriverbsimuliofdc)
CLATDOS_ddbmainbatcumaindc =  -f
LLATDOS_ddbmainbatcumaindc = 
RLATDOS_ddbmainbatcumaindc = 
BLATDOS_ddbmainbatcumaindc = 
CEAT_ddbmainbatcumaindc = $(CEAT_ts3000dexe) $(CLATDOS_ddbmainbatcumaindc)
LEAT_ddbmainbatcumaindc = $(LEAT_ts3000dexe) $(LLATDOS_ddbmainbatcumaindc)
REAT_ddbmainbatcumaindc = $(REAT_ts3000dexe) $(RLATDOS_ddbmainbatcumaindc)
BEAT_ddbmainbatcumaindc = $(BEAT_ts3000dexe) $(BLATDOS_ddbmainbatcumaindc)
CLATDOS_ddbposloopbposcondc =  -f
LLATDOS_ddbposloopbposcondc = 
RLATDOS_ddbposloopbposcondc = 
BLATDOS_ddbposloopbposcondc = 
CEAT_ddbposloopbposcondc = $(CEAT_ts3000dexe) $(CLATDOS_ddbposloopbposcondc)
LEAT_ddbposloopbposcondc = $(LEAT_ts3000dexe) $(LLATDOS_ddbposloopbposcondc)
REAT_ddbposloopbposcondc = $(REAT_ts3000dexe) $(RLATDOS_ddbposloopbposcondc)
BEAT_ddbposloopbposcondc = $(BEAT_ts3000dexe) $(BLATDOS_ddbposloopbposcondc)
CLATDOS_ddbprotocolbprotocoldc =  -f
LLATDOS_ddbprotocolbprotocoldc = 
RLATDOS_ddbprotocolbprotocoldc = 
BLATDOS_ddbprotocolbprotocoldc = 
CEAT_ddbprotocolbprotocoldc = $(CEAT_ts3000dexe) $(CLATDOS_ddbprotocolbprotocoldc)
LEAT_ddbprotocolbprotocoldc = $(LEAT_ts3000dexe) $(LLATDOS_ddbprotocolbprotocoldc)
REAT_ddbprotocolbprotocoldc = $(REAT_ts3000dexe) $(RLATDOS_ddbprotocolbprotocoldc)
BEAT_ddbprotocolbprotocoldc = $(BEAT_ts3000dexe) $(BLATDOS_ddbprotocolbprotocoldc)
CLATDOS_ddbsequbantsimdc =  -f
LLATDOS_ddbsequbantsimdc = 
RLATDOS_ddbsequbantsimdc = 
BLATDOS_ddbsequbantsimdc = 
CEAT_ddbsequbantsimdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbsequbantsimdc)
LEAT_ddbsequbantsimdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbsequbantsimdc)
REAT_ddbsequbantsimdc = $(REAT_ts3000dexe) $(RLATDOS_ddbsequbantsimdc)
BEAT_ddbsequbantsimdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbsequbantsimdc)
CLATDOS_ddbsequbantsimiodc =  -f
LLATDOS_ddbsequbantsimiodc = 
RLATDOS_ddbsequbantsimiodc = 
BLATDOS_ddbsequbantsimiodc = 
CEAT_ddbsequbantsimiodc = $(CEAT_ts3000dexe) $(CLATDOS_ddbsequbantsimiodc)
LEAT_ddbsequbantsimiodc = $(LEAT_ts3000dexe) $(LLATDOS_ddbsequbantsimiodc)
REAT_ddbsequbantsimiodc = $(REAT_ts3000dexe) $(RLATDOS_ddbsequbantsimiodc)
BEAT_ddbsequbantsimiodc = $(BEAT_ts3000dexe) $(BLATDOS_ddbsequbantsimiodc)
CLATDOS_ddbsequbseqdc =  -f
LLATDOS_ddbsequbseqdc = 
RLATDOS_ddbsequbseqdc = 
BLATDOS_ddbsequbseqdc = 
CEAT_ddbsequbseqdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbsequbseqdc)
LEAT_ddbsequbseqdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbsequbseqdc)
REAT_ddbsequbseqdc = $(REAT_ts3000dexe) $(RLATDOS_ddbsequbseqdc)
BEAT_ddbsequbseqdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbsequbseqdc)
CLATDOS_ddbsequbseqscrdc =  -f
LLATDOS_ddbsequbseqscrdc = 
RLATDOS_ddbsequbseqscrdc = 
BLATDOS_ddbsequbseqscrdc = 
CEAT_ddbsequbseqscrdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbsequbseqscrdc)
LEAT_ddbsequbseqscrdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbsequbseqscrdc)
REAT_ddbsequbseqscrdc = $(REAT_ts3000dexe) $(RLATDOS_ddbsequbseqscrdc)
BEAT_ddbsequbseqscrdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbsequbseqscrdc)
CLATDOS_ddbsequbwdogdc =  -f
LLATDOS_ddbsequbwdogdc = 
RLATDOS_ddbsequbwdogdc = 
BLATDOS_ddbsequbwdogdc = 
CEAT_ddbsequbwdogdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbsequbwdogdc)
LEAT_ddbsequbwdogdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbsequbwdogdc)
REAT_ddbsequbwdogdc = $(REAT_ts3000dexe) $(RLATDOS_ddbsequbwdogdc)
BEAT_ddbsequbwdogdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbsequbwdogdc)
CLATDOS_ddbstartrkbstartrakdc =  -f
LLATDOS_ddbstartrkbstartrakdc = 
RLATDOS_ddbstartrkbstartrakdc = 
BLATDOS_ddbstartrkbstartrakdc = 
CEAT_ddbstartrkbstartrakdc = $(CEAT_ts3000dexe) $(CLATDOS_ddbstartrkbstartrakdc)
LEAT_ddbstartrkbstartrakdc = $(LEAT_ts3000dexe) $(LLATDOS_ddbstartrkbstartrakdc)
REAT_ddbstartrkbstartrakdc = $(REAT_ts3000dexe) $(RLATDOS_ddbstartrkbstartrakdc)
BEAT_ddbstartrkbstartrakdc = $(BEAT_ts3000dexe) $(BLATDOS_ddbstartrkbstartrakdc)

#
# Dependency List
#
Dep_ts3000 = \
   ..\IDE\ts3000.exe

ts3000 : BccDos.cfg $(Dep_ts3000)
  echo MakeNode ts3000

Dep_ddbIDEbts3000dexe = \
   ..\IDE\sdc.obj\
   ..\IDE\hwio.obj\
   ..\IDE\serial.obj\
   ..\IDE\serint.obj\
   ..\IDE\serinit.obj\
   ..\IDE\tparser.obj\
   ..\IDE\tunekbd.obj\
   ..\IDE\tuneout.obj\
   ..\IDE\tunescr.obj\
   ..\IDE\comdec.obj\
   ..\IDE\intels3.obj\
   ..\IDE\simintel.obj\
   ..\IDE\kbseq.obj\
   ..\IDE\kbtask.obj\
   ..\IDE\nvrampc.obj\
   ..\IDE\screen.obj\
   ..\IDE\clock.obj\
   ..\IDE\hwpc.obj\
   ..\IDE\kbddrv.obj\
   ..\IDE\tasknull.obj\
   ..\IDE\pcscr.obj\
   ..\IDE\unos.obj\
   ..\IDE\cmclock.obj\
   ..\IDE\error.obj\
   ..\IDE\simuliof.obj\
   ..\IDE\atcumain.obj\
   ..\IDE\poscon.obj\
   ..\IDE\protocol.obj\
   ..\IDE\antsim.obj\
   ..\IDE\antsimio.obj\
   ..\IDE\seq.obj\
   ..\IDE\seqscr.obj\
   ..\IDE\wdog.obj\
   ..\IDE\startrak.obj

..\IDE\ts3000.exe : $(Dep_ddbIDEbts3000dexe)
  $(TLINK)   @&&|
 /v $(IDE_LFLAGSDOS) $(LEAT_ts3000dexe) +
C:\BC4\LIB\c0h.obj+
..\IDE\sdc.obj+
..\IDE\hwio.obj+
..\IDE\serial.obj+
..\IDE\serint.obj+
..\IDE\serinit.obj+
..\IDE\tparser.obj+
..\IDE\tunekbd.obj+
..\IDE\tuneout.obj+
..\IDE\tunescr.obj+
..\IDE\comdec.obj+
..\IDE\intels3.obj+
..\IDE\simintel.obj+
..\IDE\kbseq.obj+
..\IDE\kbtask.obj+
..\IDE\nvrampc.obj+
..\IDE\screen.obj+
..\IDE\clock.obj+
..\IDE\hwpc.obj+
..\IDE\kbddrv.obj+
..\IDE\tasknull.obj+
..\IDE\pcscr.obj+
..\IDE\unos.obj+
..\IDE\cmclock.obj+
..\IDE\error.obj+
..\IDE\simuliof.obj+
..\IDE\atcumain.obj+
..\IDE\poscon.obj+
..\IDE\protocol.obj+
..\IDE\antsim.obj+
..\IDE\antsimio.obj+
..\IDE\seq.obj+
..\IDE\seqscr.obj+
..\IDE\wdog.obj+
..\IDE\startrak.obj
$<,$*
C:\BC4\LIB\fp87.lib+
C:\BC4\LIB\mathh.lib+
C:\BC4\LIB\ch.lib

|

..\IDE\sdc.obj :  ..\iodriver\sdc.c
  $(BCCDOS) -P- -c $(CEAT_ts3000dexe) -o$@ ..\iodriver\sdc.c

..\IDE\hwio.obj :  ..\iodriver\hwio.c
  $(BCCDOS) -P- -c $(CEAT_ts3000dexe) -o$@ ..\iodriver\hwio.c

..\IDE\serial.obj :  ..\unos\serial.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbserialdc) -o$@ ..\unos\serial.c

..\IDE\serint.obj :  ..\unos\serint.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbserintdc) -o$@ ..\unos\serint.c

..\IDE\serinit.obj :  ..\unos\serinit.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbserinitdc) -o$@ ..\unos\serinit.c

..\IDE\tparser.obj :  ..\tune\tparser.c
  $(BCCDOS) -P- -c $(CEAT_ddbtunebtparserdc) -o$@ ..\tune\tparser.c

..\IDE\tunekbd.obj :  ..\tune\tunekbd.c
  $(BCCDOS) -P- -c $(CEAT_ddbtunebtunekbddc) -o$@ ..\tune\tunekbd.c

..\IDE\tuneout.obj :  ..\tune\tuneout.c
  $(BCCDOS) -P- -c $(CEAT_ddbtunebtuneoutdc) -o$@ ..\tune\tuneout.c

..\IDE\tunescr.obj :  ..\tune\tunescr.c
  $(BCCDOS) -P- -c $(CEAT_ddbtunebtunescrdc) -o$@ ..\tune\tunescr.c

..\IDE\comdec.obj :  ..\unos\comdec.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbcomdecdc) -o$@ ..\unos\comdec.c

..\IDE\intels3.obj :  ..\intelsat\intels3.c
  $(BCCDOS) -P- -c $(CEAT_ddbintelsatbintels3dc) -o$@ ..\intelsat\intels3.c

..\IDE\simintel.obj :  ..\intelsat\simintel.c
  $(BCCDOS) -P- -c $(CEAT_ddbintelsatbsiminteldc) -o$@ ..\intelsat\simintel.c

..\IDE\kbseq.obj :  ..\sequ\kbseq.c
  $(BCCDOS) -P- -c $(CEAT_ddbsequbkbseqdc) -o$@ ..\sequ\kbseq.c

..\IDE\kbtask.obj :  ..\unos\kbtask.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbkbtaskdc) -o$@ ..\unos\kbtask.c

..\IDE\nvrampc.obj :  ..\nvram\nvrampc.c
  $(BCCDOS) -P- -c $(CEAT_ddbnvrambnvrampcdc) -o$@ ..\nvram\nvrampc.c

..\IDE\screen.obj :  ..\screen\screen.c
  $(BCCDOS) -P- -c $(CEAT_ddbscreenbscreendc) -o$@ ..\screen\screen.c

..\IDE\clock.obj :  ..\unos\clock.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbclockdc) -o$@ ..\unos\clock.c

..\IDE\hwpc.obj :  ..\unos\hwpc.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbhwpcdc) -o$@ ..\unos\hwpc.c

..\IDE\kbddrv.obj :  ..\unos\kbddrv.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbkbddrvdc) -o$@ ..\unos\kbddrv.c

..\IDE\tasknull.obj :  ..\unos\tasknull.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbtasknulldc) -o$@ ..\unos\tasknull.c

..\IDE\pcscr.obj :  ..\unos\pcscr.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbpcscrdc) -o$@ ..\unos\pcscr.c

..\IDE\unos.obj :  ..\unos\unos.c
  $(BCCDOS) -P- -c $(CEAT_ddbunosbunosdc) -o$@ ..\unos\unos.c

..\IDE\cmclock.obj :  ..\clock\cmclock.c
  $(BCCDOS) -P- -c $(CEAT_ddbclockbcmclockdc) -o$@ ..\clock\cmclock.c

..\IDE\error.obj :  ..\errhand\error.c
  $(BCCDOS) -P- -c $(CEAT_ddberrhandberrordc) -o$@ ..\errhand\error.c

..\IDE\simuliof.obj :  ..\iodriver\simuliof.c
  $(BCCDOS) -P- -c $(CEAT_ddbiodriverbsimuliofdc) -o$@ ..\iodriver\simuliof.c

..\IDE\atcumain.obj :  ..\main\atcumain.c
  $(BCCDOS) -P- -c $(CEAT_ddbmainbatcumaindc) -o$@ ..\main\atcumain.c

..\IDE\poscon.obj :  ..\posloop\poscon.c
  $(BCCDOS) -P- -c $(CEAT_ddbposloopbposcondc) -o$@ ..\posloop\poscon.c

..\IDE\protocol.obj :  ..\protocol\protocol.c
  $(BCCDOS) -P- -c $(CEAT_ddbprotocolbprotocoldc) -o$@ ..\protocol\protocol.c

..\IDE\antsim.obj :  ..\sequ\antsim.c
  $(BCCDOS) -P- -c $(CEAT_ddbsequbantsimdc) -o$@ ..\sequ\antsim.c

..\IDE\antsimio.obj :  ..\sequ\antsimio.c
  $(BCCDOS) -P- -c $(CEAT_ddbsequbantsimiodc) -o$@ ..\sequ\antsimio.c

..\IDE\seq.obj :  ..\sequ\seq.c
  $(BCCDOS) -P- -c $(CEAT_ddbsequbseqdc) -o$@ ..\sequ\seq.c

..\IDE\seqscr.obj :  ..\sequ\seqscr.c
  $(BCCDOS) -P- -c $(CEAT_ddbsequbseqscrdc) -o$@ ..\sequ\seqscr.c

..\IDE\wdog.obj :  ..\sequ\wdog.c
  $(BCCDOS) -P- -c $(CEAT_ddbsequbwdogdc) -o$@ ..\sequ\wdog.c

..\IDE\startrak.obj :  ..\startrk\startrak.c
  $(BCCDOS) -P- -c $(CEAT_ddbstartrkbstartrakdc) -o$@ ..\startrk\startrak.c

# Compiler configuration file
BccDos.cfg : 
   Copy &&|
-W-
-R
-v
-vi
-X-
-H
-IC:\BC4\INCLUDE
-H=ts3setas.csm
-R
-X-
-v
-O-
-Oe
-O-b
-Z-
-k-
-vi-
-H-
-IC:\BC4\INCLUDE; ..\GLOBALH;
-H=TS3000.CSM
-n
-y
-Od
-ff-
-4
-f287
| $@


