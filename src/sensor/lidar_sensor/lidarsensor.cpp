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

#include "lidarsensor.h"

namespace SLAM{
    
    lidarSensor::lidarSensor(const std::string& name, const OrientedPoint& pose, double span, double maxRange, double angle): sensorBase(name){
        m_pose = pose;
        m_beam.span = span;
        m_beam.maxRange = maxRange;
        m_beam.pose.x = 0;
        m_beam.pose.y = 0;
        m_beam.pose.theta = angle;
        m_beam.c = cos(angle);
        m_beam.s = sin(angle);
    }
}



