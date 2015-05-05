This file is from freeby.mesanet.com/fabsread.pas
It is used reference for developing fanuc serial encoder support for Argon.



program FAbsRead;

{$IFDEF WINDOWS}
uses synaser,synautil,synaip,blcksock,dos,crt;
var
ser:TBlockSerial;
TheComPort : string;
IPAddr : string;
Socket : TUDPBlockSocket;
{$ELSE}
uses dos,crt;
var TheComPort : word;
{$ENDIF}


{$I SELECTC}
{$I SELECTIO}
{I SELECTP}
{$I SELECTPR}
{$I INTERFCE}

var

BitLength : byte;
Bitrate : real;
FAbsBaseClock : real;
BitRateDDS: longint;
FAbsBaseAddr: word;
FAbsNumregs : byte;
FAbsDataReg0 : word;
FAbsDataReg1 : word;
FAbsDataReg2 : word;
FAbsControlReg0 : word;
FAbsControlReg1 : word;
RegStride : word;
InstStride : word;

const

FAbsBusyBit : longint = $00000800;


procedure Error(err : integer);
begin
  writeln(errorrecord[err].errstr);
  halt(2);
end;

procedure GetParms;
var
retcode : integer;
begin
  BitRate := 1024000.0; { default bit rate is 1.024 M Baud }
end;

procedure SetupFAbs;
var
index : byte;
begin
  FillHM2Array;
  FAbsBaseClock := Read32(HostMotClockLowOffset);
  if not GetModuleInfo(FAbsTag,FAbsBaseAddr,FAbsNumRegs) then BumOut('No Fanuc Abs interface found');
  if not GetModuleStrides(FABsTag,RegStride,InstStride) then BumOut('No Fanuc Abs interface found');
  ZeroMasks;
  SetAllHM2OutputMasks;
  UpdateAllHM2OutputMasks;
  BitRateDDS := round(1048576*((BitRate)/FAbsBaseClock));
  FAbsDataReg0 := FAbsBaseAddr;
  FAbsDataReg1 := FAbsDataReg0+RegStride;
  FAbsDataReg2 := FAbsDataReg1+RegStride;
  FAbsControlReg0 := FAbsDataReg2+RegStride;
  FAbsControlReg1 := FAbsControlReg0+RegStride;
{  Write32(FAbsControlReg0,($00000100));
  Write32(FAbsControlReg1,(BitRateDDS or $F0000000)); }
end;

procedure BinPrint(bin: longint;count: byte);
var
index : byte;
bitmask : longint;
begin
  bitmask := (1 shl (count-1));
  for index := 1 to count do
  begin
    if (bitmask and bin) <> 0 then write('1') else write('0');
    bitmask := bitmask shr 1
  end;
end;

function SoftCRC(data0,data1,data2 : longint) : byte;
var
index,crc,cmask: byte;
bitarray :array[0..80] of boolean;
bitmask : longint;
crcreg,oldcrcreg : array[0..4] of boolean;

begin
  {This first part just makes an array of booleans for easy manipulation }

  for index := 0 to 80  do bitarray[index] := false;
  bitmask := 1;
  for index := 0 to 31 do
  begin
  if (bitmask and data0) <> 0 then
    begin
      bitarray[index] := true;
    end;
    bitmask := bitmask shl 1;
  end;
  bitmask := 1;
  for index := 0 to 31 do
  begin
  if (bitmask and data1) <> 0 then
    begin
      bitarray[index+32] := true;
    end;
    bitmask := bitmask shl 1;
  end;
  bitmask := 1;
  for index := 0 to 11 do
  begin
  if (bitmask and data2) <> 0 then
    begin
      bitarray[index+64] := true;
    end;
    bitmask := bitmask shl 1;
  end;
  {this is the actual bit oriented CRC5-ITU routine}
  for index := 0 to 4 do crcreg[index] := false;
  for index := 0 to 4 do oldcrcreg[index] := false;

  for index := 0 to 75 do  { CRC-5 ITU }
  begin
    crcreg[0] := bitarray[75-index] xor oldcrcreg[4];  { CRC calculated MSB first }
    crcreg[1] := oldcrcreg[0];
    crcreg[2] := oldcrcreg[1] xor bitarray[75-index] xor oldcrcreg[4];
    crcreg[3] := oldcrcreg[2];
    crcreg[4] := oldcrcreg[3] xor bitarray[75-index] xor oldcrcreg[4];
    oldcrcreg := crcreg;
  end;
  crc := 0;
  cmask := 1;
  {convert remainder back to a number}
  for index := 0 to 4 do
  begin
    if crcreg[index] = true then crc := crc or cmask;
    cmask := cmask shl 1;
  end;
  SoftCRC := crc;
end;


procedure FAbsReadLoop;
var
data0,data1,data2 : longint;
crc,calccrc : byte;
count : longint;
lcount : longint;
turns : longint;
comabs : longint;
timeout : longint;
begin
  ClrScr;
  while not keypressed do
  begin
    Write32(FAbsDataReg0,0);
    timeout := 100;
    while((Read32(FAbsControlReg0) and FAbsBusyBit) <> 0) and (timeout <> 0) do timeout := timeout-1;
    GotoXY(1,4);
    if timeout = 0 then
    begin
      write('Encoder timeout');
      delay(10);
      ClrScr;
    end
    else
    begin
      GotoXY(1,5);
      data0 := Read32(FAbsDataReg0);
      data1 := Read32(FAbsDataReg1);
      data2 := Read32(FAbsDataReg2);
      if (data2 and $80000000) <> 0 then
      begin
        write ('Cable error');
        delay(10);
        ClrScr;
      end
      else
      begin
        hexprint(data2,8);
        hexprint(data1,8);
        hexprint(data0,8);
        GotoXY(1,6);
        binprint(data2,12);
        binprint(data1,32);
        binprint(data0,32);
        GotoXY(1,7);
        write('7777776666666666555555555544444444443333333333222222222211111111110000000000');
        GotoXY(1,8);
        write('5432109876543210987654321098765432109876543210987654321098765432109876543210');
        crc := data2 shr 7;
        GotoXY(1,9);
        write('CRC = ');
        hexprint(crc,2);
        calccrc := SoftCRC(data0,data1,data2);
        write(' CRC Remainder = ');
        hexprint(calccrc,2);
        if calccrc <> 0 then
        begin
          write(' CRC-ERROR ',chr(7));
          delay(100);
        end;
        write(' Index det: ');
        if (data0 and $100) = 0 then write('true ') else write('false');
        write(' Bat det: ');
        if (data0 and $20) = 0 then write('true ') else write('false');
        write('            ');
        GotoXY(1,10);
        write('Count = ');
        count := (data0 shr 18) + ((data1 and 3) shl 14);
        write(count:6);
        write(' LCount = ');
        lcount := count*16 + (data0 shr 12) and $F;
        write(lcount:7);
        turns := (data1 shr 4) and $0000FFFF;
        write(' Turns = ');
        write(turns:6);
        comabs := (data1 shr 22);
        write(' ComAbs = ');
        write(comabs:4);
      end;
    end;
  end;
  readkey;
end;

begin
  GetOurEnv;
  if not InitializeInterface(message) then bumout(message);
  GetParms;
  SetupFAbs;
  FAbsReadLoop;
end.
