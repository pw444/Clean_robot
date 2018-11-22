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

#ifndef SLAM_ARRAY2D_H
#define SLAM_ARRAY2D_H

#include "../utils/point.h"
#include "accessstatus.h"

namespace SLAM {

template<class Cell>
class array2D
{
public:
    array2D(int xsize = 0,int ysize = 0);
    array2D& operator= (const array2D&);
    array2D(const array2D<Cell> &);
    ~array2D();
    void clear();
    void resize(int n_xsize, int n_ysize);
    
    bool isInside(int x, int y);
    Cell& cell(int x, int y);
    accessStatus cellState(int x, int y);
    
    bool isInside(const IntPoint &);
    Cell& cell(const IntPoint &);
    accessStatus cellState(const IntPoint &);
    
    inline int getXSize() const { return m_xsize;}
    inline int getYSize() const { return m_ysize;}
    Cell ** cells;
protected:
    int m_xsize;
    int m_ysize;
};

    template <class Cell> 
    array2D<Cell>::array2D(int xsize,int ysize){
        if(xsize > 0 && ysize > 0){
            m_xsize = xsize;
            m_ysize = ysize;
            cells = new Cell*[m_xsize];
            for(int i =0; i < m_xsize; i++){
                cells[i] = new Cell[m_ysize];
            }
        }else{
            m_xsize = m_ysize = 0;
            cells = 0;
        }
    }
    
   template<class Cell>
   array2D<Cell>::~array2D(){
       for(int i = 0; i < m_xsize; i++){
           delete[] cells[i];
           cells[i] = 0;
       }
       delete cells;
   }
   
   template<class Cell> 
    array2D<Cell> & array2D<Cell>::operator=(const array2D<Cell>& a){
       if(a.m_xsize != m_xsize || a.m_ysize != m_ysize){
           for(int i = 0; i < m_xsize; i++){
               delete[] cells[i];
           }
           delete[] cells;
           m_xsize = a.m_xsize;
           m_ysize = a.m_ysize;
           cells = new Cell*[m_xsize];
           for(int i = 0; i < m_xsize; i++){
               cells[i] = new Cell[m_ysize];
           }
       }
       for(int i = 0; i < m_xsize; i++)
           for(int j =0; j < m_ysize; j++)
               cells[i][j] = a.cells[i][j];
        return *this;
   }
   
   template<class Cell>
   array2D<Cell>::array2D(const array2D<Cell>& a){
       m_xsize = a.m_xsize;
       m_ysize = a.m_ysize;
       cells = new Cell*[m_xsize];
       for(int i = 0; i < m_xsize; i++){
           cells[i] =  new Cell[m_ysize];
           for(int j=0; j < m_ysize; j++){
               cells[i][j] = a.cells[i][j];
           }
       }
   }
   
   
   template<class Cell>
   void array2D<Cell>::clear(){
       for(int i = 0; i < m_xsize; i++){
           delete[] cells[i];
           cells[i] = 0;
       }
       delete cells;
       cells = 0;
       m_xsize = 0;
       m_ysize = 0;
   }
       
   
   template<class Cell>
   void array2D<Cell>::resize(int n_xsize, int n_ysize){
       Cell** new_cells = new Cell* [n_xsize];
       for(int i = 0; i < n_xsize; i++){
           new_cells[i] = new Cell[n_ysize];
       }
       int x_min = n_xsize < m_xsize ? n_xsize : m_xsize;
       int y_min = n_ysize < m_ysize ? n_ysize : m_ysize;
       for(int i = 0; i < x_min; i++){
           for(int j = 0; j < y_min; j++){
               new_cells[i][j] = cells[i][j];
           }
           delete[] cells[i];
       }
       delete[] cells;
       cells = new_cells;
       m_xsize = n_xsize;
       m_ysize = n_ysize;
   }
   
   template<class Cell>
   bool array2D<Cell>::isInside(int x, int y){
       return x>=0 && x<m_xsize && y>=0 && y<m_ysize;
   }
   
   template<class Cell>
   bool array2D<Cell>::isInside(const IntPoint& p){
       return isInside(p.x, p.y);
   }
   
   template<class Cell>
   Cell& array2D<Cell>::cell(int x, int y){
       if(isInside(x,y)){
           return cells[x][y];
       }else{
           return cells[0][0];
       }
   }
   
   template<class Cell>
   Cell& array2D<Cell>::cell(const IntPoint& p){
       return cell(p.x, p.y);
   }
   
   template<class Cell>
   accessStatus array2D<Cell>::cellState(int x, int y){
       return (accessStatus) (isInside(x,y) ? (Inside | Allocated) : Outside);
   }
   
   template<class Cell>
   accessStatus array2D<Cell>::cellState(const IntPoint& p){
       return cellState(p.x, p.y);
   }

}

#endif // SLAM_ARRAY2D_H
