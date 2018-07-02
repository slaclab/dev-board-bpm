-------------------------------------------------------------------------------
-- File       : AppCoreConfigPkg.vhd
-- Company    : SLAC National Accelerator Laboratory
-------------------------------------------------------------------------------
-- Description: AppCore configuration for BPM
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

use work.AppCorePkg.all;

package AppCoreConfigPkg is

   function appCoreConfigFunc return AppCoreConfigType;

   constant APP_CORE_CONFIG_C : AppCoreConfigType := appCoreConfigFunc;

end package AppCoreConfigPkg;

package body AppCoreConfigPkg is

   function appCoreConfigFunc return AppCoreConfigType is
      variable v : AppCoreConfigType;
   begin
      v := APP_CORE_CONFIG_DFLT_C;
      v.numBays := 2;
      return v;
   end function appCoreConfigFunc;
   
end package body AppCoreConfigPkg;
