/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2018  <copyright holder> <email>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef SLAM_LIDARSENSOR_H
#define SLAM_LIDARSENSOR_H

#include "sensor/sensor_base/sensorbase.h"
#include "utils/point.h"
#include <vector>

namespace SLAM {
    
    class lidarSensor: public sensorBase
    {
    public:
        struct Beam{
            OrientedPoint pose;
            double span;
            double maxRange;
            double s,c;
        };
        
        lidarSensor(const std::string& name=""):sensorBase(name) {};
        lidarSensor(const std::string& name, const OrientedPoint& pose = OrientedPoint(0,0,0), double span = 0, double maxRange = 0, double angle = 0);
        inline OrientedPoint getPose() const { return m_pose; }
        inline Beam getBeam() { return m_beam; }
    protected:
        OrientedPoint m_pose;
        Beam m_beam;
    };

}

#endif // SLAM_LIDARSENSOR_H
