-------------------------------------------------------------------------------
-- File       : AppCore.vhd
-- Company    : SLAC National Accelerator Laboratory
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------
-- This file is part of 'Example Project Firmware'.
-- It is subject to the license terms in the LICENSE.txt file found in the
-- top-level directory of this distribution and at:
--    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
-- No part of 'Example Project Firmware', including this file,
-- may be copied, modified, propagated, or distributed except according to
-- the terms contained in the LICENSE.txt file.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.StdRtlPkg.all;
use work.AxiLitePkg.all;
use work.AxiStreamPkg.all;
use work.SsiPkg.all;
use work.TimingPkg.all;
use work.AmcCarrierPkg.all;
use work.Jesd204bPkg.all;
use work.AppTopPkg.all;
use work.AppCorePkg.all;
use work.AppCoreConfigPkg.all;
use work.BpmPkg.all;

architecture Impl of AppCore is

   type     ModeArray is array (natural range <>) of natural;

   constant BAY_0_C            : natural :=  0;
   constant BAY_1_C            : natural :=  1;
   constant NUM_BAYS_C         : natural :=  2;
   constant NUM_CFG_BAYS_C     : natural :=  APP_CORE_CONFIG_G.numBays;

   constant BPM_MODE_C : ModeArray(NUM_BAYS_C - 1 downto 0) := (
      BAY_0_C => 0, -- stripline
      BAY_1_C => 1  -- cavity
   );

   constant S_SBPM0_INDEX_C    : natural :=  0;
   constant S_SBPM1_INDEX_C    : natural :=  1;
   constant LINSIM0_INDEX_C    : natural :=  2;
   constant LINSIM1_INDEX_C    : natural :=  3;
   constant SIMFEED0_INDEX_C   : natural :=  4;
   constant SIMFEED1_INDEX_C   : natural :=  5;
   constant LOC_CTRL_INDEX_C   : natural :=  6;

   constant NUM_AXI_MASTERS_C  : natural :=  7;

   constant NUM_AXI_SLAVES_C   : natural := 1 + NUM_BAYS_C;

   constant NUM_LINSIM_CHANNELS_C : integer := 4;
   constant LINSIM_TDATA_BYTES_C  : integer := 8; -- 4 or 8; depending on NUM_LINSIM_CHANNELS_C

   constant NUM_LOCAL_RW_REGS_C   : integer := 4;
   constant NUM_LOCAL_RO_REGS_C   : integer := 5;

   function "+"(a,b: slv) return slv is
   begin
      return slv( unsigned(a) + unsigned(b) );
   end function "+";

   constant AXI_CROSSBAR_MASTERS_CONFIG_C : AxiLiteCrossbarMasterConfigArray(NUM_AXI_MASTERS_C-1 downto 0) := (
      S_SBPM0_INDEX_C => (
         baseAddr     => AXIL_BASE_ADDR_G + x"0000_0000",
         addrBits     => 16,
         connectivity => X"FFFF"),
      S_SBPM1_INDEX_C => (
         baseAddr     => AXIL_BASE_ADDR_G + x"0001_0000",
         addrBits     => 16,
         connectivity => X"FFFF"),
      LINSIM0_INDEX_C=> (
         baseAddr     => AXIL_BASE_ADDR_G + x"0002_0000",
         addrBits     => 16,
         connectivity => X"FFFF"),
      LINSIM1_INDEX_C => (
         baseAddr     => AXIL_BASE_ADDR_G + x"0003_0000",
         addrBits     => 16,
         connectivity => X"FFFF"),
      SIMFEED0_INDEX_C=> (
         baseAddr     => AXIL_BASE_ADDR_G + x"0004_0000",
         addrBits     => 16,
         connectivity => X"FFFF"),
      SIMFEED1_INDEX_C=> (
         baseAddr     => AXIL_BASE_ADDR_G + x"0005_0000",
         addrBits     => 16,
         connectivity => X"FFFF"),
      LOC_CTRL_INDEX_C=> (
         baseAddr     => AXIL_BASE_ADDR_G + x"000F_0000",
         addrBits     => 16,
         connectivity => X"FFFF")
   );

   signal mAxilWriteMasters : AxiLiteWriteMasterArray(NUM_AXI_MASTERS_C-1 downto 0);
   signal mAxilWriteSlaves  : AxiLiteWriteSlaveArray (NUM_AXI_MASTERS_C-1 downto 0) := (others => AXI_LITE_WRITE_SLAVE_EMPTY_DECERR_C);
   signal mAxilReadMasters  : AxiLiteReadMasterArray (NUM_AXI_MASTERS_C-1 downto 0);
   signal mAxilReadSlaves   : AxiLiteReadSlaveArray  (NUM_AXI_MASTERS_C-1 downto 0) := (others => AXI_LITE_READ_SLAVE_EMPTY_DECERR_C);

   signal simFeedObAxilWrMst: AxiLiteWriteMasterArray(NUM_BAYS_C - 1 downto 0) := (others => AXI_LITE_WRITE_MASTER_INIT_C);
   signal simFeedObAxilWrSlv: AxiLiteWriteSlaveArray (NUM_BAYS_C - 1 downto 0);
   signal simFeedObAxilRdMst: AxiLiteReadMasterArray (NUM_BAYS_C - 1 downto 0) := (others => AXI_LITE_READ_MASTER_INIT_C);
   signal simFeedObAxilRdSlv: AxiLiteReadSlaveArray  (NUM_BAYS_C - 1 downto 0);

   signal bpmBus            : BpmBusArray := (others => BPM_BUS_INIT_C);
   signal locDiagnostic     : DiagnosticBusType;

   signal bpMsgTimestamp    : slv(63 downto 0);

   signal bpMsgDecimated    : sl;
   signal bpMsgDecimatorEn  : sl := '0';
   signal bpMsgStrobe       : sl;
   signal bpMsgRateMonRst   : sl;

   signal locCtrlRegVals    : Slv32Array(NUM_LOCAL_RW_REGS_C-1 downto 0);
   signal locStatRegVals    : Slv32Array(NUM_LOCAL_RO_REGS_C-1 downto 0);

   signal mBpmDataStream    : AxiStreamMasterArray(NUM_BAYS_C - 1 downto 0) := (others => AXI_STREAM_MASTER_INIT_C);
   signal sBpmDataStream    : AxiStreamSlaveArray (NUM_BAYS_C - 1 downto 0);

begin


   ---------------------------
   -- AXI-Lite Crossbar Module
   ---------------------------
   U_XBAR : entity work.AxiLiteCrossbar
      generic map (
         TPD_G              => TPD_G,
         NUM_SLAVE_SLOTS_G  => NUM_AXI_SLAVES_C,
         NUM_MASTER_SLOTS_G => NUM_AXI_MASTERS_C,
         MASTERS_CONFIG_G   => AXI_CROSSBAR_MASTERS_CONFIG_C)
      port map (
         sAxiWriteMasters(0) => sAxilWriteMaster,
         sAxiWriteMasters(1) => simFeedObAxilWrMst(0),
         sAxiWriteMasters(2) => simFeedObAxilWrMst(1),
         sAxiWriteSlaves (0) => sAxilWriteSlave,
         sAxiWriteSlaves (1) => simFeedObAxilWrSlv(0),
         sAxiWriteSlaves (2) => simFeedObAxilWrSlv(1),
         sAxiReadMasters (0) => sAxilReadMaster,
         sAxiReadMasters (1) => simFeedObAxilRdMst(0),
         sAxiReadMasters (2) => simFeedObAxilRdMst(1),
         sAxiReadSlaves  (0) => sAxilReadSlave,
         sAxiReadSlaves  (1) => simFeedObAxilRdSlv(0),
         sAxiReadSlaves  (2) => simFeedObAxilRdSlv(1),
         mAxiWriteMasters    => mAxilWriteMasters,
         mAxiWriteSlaves     => mAxilWriteSlaves,
         mAxiReadMasters     => mAxilReadMasters,
         mAxiReadSlaves      => mAxilReadSlaves,
         axiClk              => axilClk,
         axiClkRst           => axilRst);

   GEN_BAYS : for bay in NUM_CFG_BAYS_C - 1 downto 0 generate

      signal sampleDataA       : slv(31 downto 0);
      signal sampleDataB       : slv(31 downto 0);
      signal sampleDataC       : slv(31 downto 0);
      signal sampleDataD       : slv(31 downto 0);

      signal sampleDataValid   : slv( 3 downto 0);

      signal jesdTrigger       : sl;
      signal trig              : sl;
      signal fastTrig          : sl;
      signal slowTrig          : sl;

   begin

   U_LinSim : entity work.RxChannelsAdapter
      generic map (
         NUM_CHANNELS_G       => NUM_LINSIM_CHANNELS_C,
         NUM_POLE_ZERO_SECS_G => 3,
         NUM_POLE_ONLY_SECS_G => 5,
         WS2_DELAY_G          => 2 -- one less than the clock multiplication factor jesdUsrClk -> jesdClk2x
      )
      port map (

         fastStreamingClk   => jesdClk2x(bay),
         fastStreamingRst   => jesdRst2x(bay),

         halfStreamingClk   => jesdClk(bay),
         halfStreamingRst   => jesdRst(bay),

         jesdTrigger        => jesdTrigger,

         sampleDataA        => sampleDataA,
         sampleDataB        => sampleDataB,
         sampleDataC        => sampleDataC,
         sampleDataD        => sampleDataD,

         sampleDataValid    => sampleDataValid,

		 -- runningOut out sl
         trigOut            => trig,

         slowRegClk         => jesdUsrClk(bay),
         slowRegRst         => jesdUsrRst(bay),

         axilClk            => axilClk,
         axilRst            => axilRst,

         axiLiteReadMaster  => mAxilReadMasters (LINSIM0_INDEX_C + bay),
         axiLiteReadSlave   => mAxilReadSlaves  (LINSIM0_INDEX_C + bay),
         axiLiteWriteMaster => mAxilWriteMasters(LINSIM0_INDEX_C + bay),
         axiLiteWriteSlave  => mAxilWriteSlaves (LINSIM0_INDEX_C + bay)

      );

   debugValids(bay)    <= sampleDataValid;
   debugValues(bay, 0) <= sampleDataA;
   debugValues(bay, 1) <= sampleDataB;
   debugValues(bay, 2) <= sampleDataC;
   debugValues(bay, 3) <= sampleDataD;

   slowTrig        <= timingTrig.trigPulse(0 + bay);
   fastTrig        <= timingTrig.trigPulse(4 + bay) or timingTrig.trigPulse(6 + bay);

   trigHw(bay) <= slowTrig or timingTrig.trigPulse(6 + bay);

   U_BpmCoreWrapper : entity work.BpmCoreWrapper
      generic map (
         MODE_CAVITY_BPM_G  => BPM_MODE_C(bay)
      )
      port map (
         jesdClk2x          => jesdClk2x(bay),
         jesdRst2x          => jesdRst2x(bay),

         jesdClk            => jesdClk(bay),
         jesdRst            => jesdRst(bay),

         jesdTriggerOut     => jesdTrigger,

         sampleDataA        => sampleDataA,
         sampleDataB        => sampleDataB,
         sampleDataC        => sampleDataC,
         sampleDataD        => sampleDataD,
         sampleDataValid    => sampleDataValid,

         axilClk            => axilClk,
         axilRst            => axilRst,

         axiLiteReadMaster  => mAxilReadMasters (S_SBPM0_INDEX_C + bay),
         axiLiteReadSlave   => mAxilReadSlaves  (S_SBPM0_INDEX_C + bay),
         axiLiteWriteMaster => mAxilWriteMasters(S_SBPM0_INDEX_C + bay),
         axiLiteWriteSlave  => mAxilWriteSlaves (S_SBPM0_INDEX_C + bay),

         calTrigger         => '0',
         calGrn             => '0',

         axiStreamMasterOb  => mBpmDataStream(bay),
         axiStreamSlaveOb   => sBpmDataStream(bay),

         timingClk          => timingClk,
         timingRst          => timingRst,

         timingBus          => timingBus,

         fastTrigger        => fastTrig,
         slowTrigger        => slowTrig,

         bpmBus             => bpmBus(bay)
      );

   U_SimFeed : entity work.SimSrcWrapper
      generic map (
         TPD_G              => TPD_G,
         LD_SLOTS_G         => 6, -- 64 beam pulses
         INIT_G             => resize(x"08000", 18),
         ADDRS_G            => (  -- QSCL register address in simulator
                                 0 => (AXI_CROSSBAR_MASTERS_CONFIG_C(LINSIM0_INDEX_C + bay).baseAddr + x"00001FE8"),
                                 1 => (AXI_CROSSBAR_MASTERS_CONFIG_C(LINSIM0_INDEX_C + bay).baseAddr + x"00002FE8"),
                                 2 => (AXI_CROSSBAR_MASTERS_CONFIG_C(LINSIM0_INDEX_C + bay).baseAddr + x"00003FE8"),
                                 3 => (AXI_CROSSBAR_MASTERS_CONFIG_C(LINSIM0_INDEX_C + bay).baseAddr + x"00004FE8")
                               )
      )
      port map (
         axilClk            => axilClk,
         axilRst            => axilRst,

         axilReadMasterIb   => mAxilReadMasters (SIMFEED0_INDEX_C + bay),
         axilReadSlaveIb    => mAxilReadSlaves  (SIMFEED0_INDEX_C + bay),
         axilWriteMasterIb  => mAxilWriteMasters(SIMFEED0_INDEX_C + bay),
         axilWriteSlaveIb   => mAxilWriteSlaves (SIMFEED0_INDEX_C + bay),

         axilWriteMasterOb  => simFeedObAxilWrMst( bay ),
         axilWriteSlaveOb   => simFeedObAxilWrSlv( bay ),
         axilReadMasterOb   => simFeedObAxilRdMst( bay ),
         axilReadSlaveOb    => simFeedObAxilRdSlv( bay ),

         timingClk          => timingClk,
         timingRst          => timingRst,
         timingBus          => timingBus,

         strobeClk          => jesdClk2x(bay),
         strobe             => trig
      );

   end generate;

   ----------------------
   -- Compose BSA/BLEN Message
   ----------------------
   U_MsgComposer : entity work.MessageComposer
      generic map (
         TPD_G        => TPD_G
      )
      port map (
         timingClk     => timingClk,
         timingRst     => timingRst,

         doPost        => timingTrig.trigPulse(11),

         timingBus     => timingBus,

         bpmBus        => bpmBus,

         diagnosticBus => locDiagnostic
      );

   diagnosticClk     <= timingClk;
   diagnosticRst     <= timingRst;
   diagnosticBus     <= locDiagnostic;

   bpMsgTimestamp    <= ite( BPM_MSG_TSNOTPID_C, locDiagnostic.timingMessage.timeStamp, locDiagnostic.timingMessage.pulseID );

   U_SYNC_ENBDEC : entity work.Synchronizer
      generic map (
         TPD_G   => TPD_G
      )
      port map (
         clk     => timingClk,
         rst     => timingRst,
         dataIn  => locCtrlRegVals(2)(0),
         dataOut => bpMsgDecimatorEn
      );


   P_BpMsgDecimator : process (bpMsgDecimatorEn, timingTrig.trigPulse(12)) is
   begin
      if ( bpMsgDecimatorEn = '1' ) then
         -- trigger pulse must fire synchronously with lcls2TrigPulse(11) but last for at least 2 cycles
         bpMsgDecimated <= timingTrig.trigPulse(12);
      else
         bpMsgDecimated <= '1';
      end if;
   end process P_BpMsgDecimator;

   bpMsgRateMonRst <= axilRst or locCtrlRegVals(2)(1);

   U_BpmMsgDecimatorMon : entity work.SyncTrigRate
      generic map (
         TPD_G               => TPD_G,
         REF_CLK_FREQ_G      => AXIL_CLK_FRQ_G
      )
      port map (
         locClk              => timingClk,
         locRst              => timingRst,

         trigIn              => bpMsgStrobe,

         refClk              => axilClk,
         refRst              => bpMsgRateMonRst,
         trigRateOut         => locStatRegVals(2),
         trigRateOutMin      => locStatRegVals(3),
         trigRateOutMax      => locStatRegVals(4)
      );

   bpMsgStrobe <= locDiagnostic.strobe and bpMsgDecimated;

   U_BpMsgOb : entity work.AppMsgOb
      generic map (
         TPD_G       => TPD_G,
         HDR_SIZE_G  => BPM_MSG_HDR_SIZE_C, -- Needs to be the same as BLEN
         EN_CRC_G    => BPM_MSG_CRC_ENBL_C,
         DATA_SIZE_G => BPM_N_MSG_CHANNELS) -- Needs to be the same as BLEN
      port map (
         -- Application Messaging Interface (clk domain)
         clk         => timingClk,
         rst         => timingRst,
         strobe      => bpMsgStrobe,                                       -- to BLEN
         header      => (others => (others => '0')),                       -- to BLEN
         timeStamp   => bpMsgTimestamp,                                    -- to BLEN
         data        => locDiagnostic.data(BPM_N_MSG_CHANNELS-1 downto 0), -- to BLEN
         -- Backplane Messaging Interface  (axilClk domain)
         axilClk     => axilClk,
         axilRst     => axilRst,
         obMsgMaster => obAxisMasters(APP_BPCLT_STRM_C),
         obMsgSlave  => obAxisSlaves (APP_BPCLT_STRM_C)
      );

   U_StreamMux : entity work.AxiStreamMux
      generic map (
         TPD_G              => TPD_G,
         NUM_SLAVES_G       => 2,
         MODE_G             => "ROUTED",
         TDEST_ROUTES_G     => ( 1 => x"C1", 0 => x"C0" ))
      port map (
         axisClk            => axilClk,
         axisRst            => axilRst,

         sAxisMasters(0)    => mBpmDataStream(0),
         sAxisMasters(1)    => mBpmDataStream(1),

         sAxisSlaves(0)     => sBpmDataStream(0),
         sAxisSlaves(1)     => sBpmDataStream(1),

         mAxisMaster        => obAxisMasters(APP_DEBUG_STRM_C),
         mAxisSlave         => obAxisSlaves (APP_DEBUG_STRM_C)
      );


   U_LocCtrl : entity work.AxiLiteRegs
      generic map (
         TPD_G              => TPD_G,
         NUM_WRITE_REG_G    => NUM_LOCAL_RW_REGS_C,
         NUM_READ_REG_G     => NUM_LOCAL_RO_REGS_C
      )
      port map (
         axiClk             => axilClk,
         axiClkRst          => axilRst,

         axiReadMaster      => mAxilReadMasters ( LOC_CTRL_INDEX_C ),
         axiReadSlave       => mAxilReadSlaves  ( LOC_CTRL_INDEX_C ),
         axiWriteMaster     => mAxilWriteMasters( LOC_CTRL_INDEX_C ),
         axiWriteSlave      => mAxilWriteSlaves ( LOC_CTRL_INDEX_C ),

         writeRegister      => locCtrlRegVals,
         readRegister       => locStatRegVals
      );

   locStatRegVals(0) <= (others => '0');
   locStatRegVals(1) <= (others => '0');

   appLeds(0) <= bpmBus(BAY_0_C).debug(0);
   appLeds(1) <= bpmBus(BAY_0_C).debug(1);

end Impl;
