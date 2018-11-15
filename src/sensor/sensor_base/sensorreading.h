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

#ifndef SLAM_SENSORREADING_H
#define SLAM_SENSORREADING_H

#include "sensorbase.h"

namespace SLAM {

    class sensorReading
    {
    public:
        sensorReading(const sensorBase* sensor=0, const double time): m_sensor(sensor), m_time(time) {};
        virtual ~sensorReading() {};
        
        inline void setSensor(const sensorBase* sensor) { m_sensor = sensor; }
        inline void setTime(const double time) { m_time = time; }
        inline sensorBase* getSensor() { return m_sensor; }
        inline double getTime() { return m_time; }
    protected:
        sensorBase* m_sensor;
        double m_time;
    };

}

#endif // SLAM_SENSORREADING_H
