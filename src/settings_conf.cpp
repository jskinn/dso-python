/**
* This file is part of DSO.
* 
* Copyright 2019 John Skinner.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/
#include "util/settings.h"
#include "settings_conf.h"


namespace dso
{

void configure(int preset, int mode, bool quiet, bool nolog)
{
    // Set up some global settings. This is copied in fragments from main_dso_pangolin.cpp
    // Preset settings
    if(preset == 0 || preset == 1)
	{
		setting_desiredImmatureDensity = 1500;
		setting_desiredPointDensity = 2000;
		setting_minFrames = 5;
		setting_maxFrames = 7;
		setting_maxOptIterations=6;
		setting_minOptIterations=1;

		setting_logStuff = false;
	}
	else if(preset == 2 || preset == 3)
	{
		setting_desiredImmatureDensity = 600;
		setting_desiredPointDensity = 800;
		setting_minFrames = 4;
		setting_maxFrames = 6;
		setting_maxOptIterations=4;
		setting_minOptIterations=1;

		benchmarkSetting_width = 424;
		benchmarkSetting_height = 320;
	}

    // Logging and quiet settings
    setting_logStuff = !nolog;
    setting_debugout_runquiet = quiet;

    // Mode settings, set to 0 for full photometric calibration
    if(mode == 1)
    {
        // printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
        setting_photometricCalibration = 0;
        setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    }
    else if(mode ==2)
    {
        //printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
        setting_photometricCalibration = 0;
        setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
        setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
        setting_minGradHistAdd=3;
    }
}

}