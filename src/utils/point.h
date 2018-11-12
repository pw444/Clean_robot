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
 *
 */

#ifndef POINT_H
#define POINT_H

#include<math.h>
#include<iostream>
#include<values.h>

namespace SLAM {
     template <class T>
     struct point {
         inline point():x(0),y(0) {}
         inline point(T _x, T _y):x(_x),y(_y) {}
         T x,y;
     };
     
     template <class T>
     inline point<T> operator + (const point<T>& p1, const point<T>& p2){
         return point<T>(p1.x+p2.x, p1.y+p2.y);
     };
     
     template <class T>
     inline point<T> operator - (const point<T>& p1, const point<T>& p2){
         return point<T>(p1.x-p2.x, p1.y-p2.y);
     };
     
     template <class T>
     inline point<T> operator * (const point<T>& p1, const T &v){
         return point<T>(p1.x*v, p1.y*v);
     };
     
     template <class T>
     inline point<T> operator * (const T &v, const point<T>& p1){
         return point<T>(p1.x*v, p1.y*v);
     };
     

     template <class T>
     inline T operator * (const point<T>& p1, const point<T>& p2){
         return p1.x*p2.x+p1.y*p2.y;
     };
     
     template <class T, class P>
     struct orientedPoint: public point<T> {
         inline orientedPoint(): point<T>(0,0), theta(0){};
         inline orientedPoint(const point<T>& p): point<T>(p.x,p.y), theta(0){};
         inline orientedPoint(T _x, T _y, P _a): point<T>(_x,_y), theta(_a){};
         inline void normalize();
         inline orientedPoint<T,P> rotate(P alpha);
         P theta;
     };
     
     template <class T, class P>
     void orientedPoint<T,P>::normalize(){
         if (theta >= -M_PI && theta <= M_PI)
             return;
         
         int count = (int)(theta / (2 * M_PI));
         theta = theta - count * 2 * M_PI;
         if(theta >= M_PI)
             theta -= 2*M_PI;
         if(theta <= -M_PI)
             theta += 2*M_PI;
     };
     
     template <class T, class P>
     orientedPoint<T,P> orientedPoint<T,P>::rotate(P alpha){
         T s = sin(alpha), c = cos(alpha);
         P angle = alpha + theta;
         angle = atan2(sin(angle),cos(angle));
         return orientedPoint(
             c*this->x-s*this->y,
             s*this->x+c*this->y,
             angle);
     };
     
     template <class T, class P>
     inline orientedPoint<T,P> operator + (const orientedPoint<T,P> &o1, const orientedPoint<T,P> &o2){
         return orientedPoint<T,P>(o1.x+o2.x, o1.y+o2.y, o1.theta+o2.theta);
     };
     
     template <class T, class P>
     inline orientedPoint<T,P> operator - (const orientedPoint<T,P> &o1, const orientedPoint<T,P> &o2){
         return orientedPoint<T,P>(o1.x-o2.x, o1.y-o2.y, o1.theta-o2.theta);  
     };
     
     template <class T, class P>
     inline orientedPoint<T,P> operator * (const T& v, const orientedPoint<T,P> &o){
         return orientedPoint<T,P>(o.x*v, o.y*v, o.theta*v);
     };
     
     template <class T, class P>
     inline orientedPoint<T,P> operator * (const orientedPoint<T,P> &o, const T& v){
         return orientedPoint<T,P>(o.x*v, o.y*v, o.theta*v);
     };
     
     template <class T>
     inline point<T> max(const point<T> &p1, const point<T> &p2){
         point<T> p;
         p.x = p1.x>p2.x ? p1.x : p2.x;
         p.y = p1.y>p2.y ? p1.y : p2.y;
         return p;
     };
     
     template <class T>
     inline point<T> min(const point<T> &p1, const point<T> &p2){
         point<T> p;
         p.x = p1.x<p2.x ? p1.x : p2.x;
         p.y = p1.y<p2.y ? p1.y : p2.y;
         return p;
     };
     
     template <class T>
     inline double euclidianDist(const point<T> &p1, const point<T> &p2){
         return hypot(p1.x-p2.x, p1.y-p2.y);
     };
     
     template <class T, class P>
     inline double euclidianDist(const orientedPoint<T,P> &o1, const orientedPoint<T,P> &o2){
         return hypot(o1.x-o2.x, o1.y-o2.y);
     };
     
     template <class T, class P>
     inline double euclidianDist(const point<T> &p, const orientedPoint<T,P> &o){
         return hypot(p.x-o.x, p.y-o.y);
     };
     
     template <class T, class P>
     inline double euclidianDist(const orientedPoint<T,P> &o, const point<T> &p){
         return hypot(o.x-p.x, o.y-p.y);
     };
     
     typedef point<int> IntPoint;
     typedef point<double> DoublePoint;
     typedef orientedPoint<double, double> OrientedPoint;
}
#endif // POINT_H
