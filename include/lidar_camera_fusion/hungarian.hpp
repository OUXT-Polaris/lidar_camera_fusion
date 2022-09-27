// Copyright (c) 2016 Cong Ma
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <iostream>
#include <vector>

class HungarianAlgorithm
{
public:
  HungarianAlgorithm();
  ~HungarianAlgorithm();
  double Solve(std::vector<std::vector<double> > & DistMatrix, std::vector<int> & Assignment);

private:
  void assignmentoptimal(
    int * assignment, double * cost, double * distMatrix, int nOfRows, int nOfColumns);
  void buildassignmentvector(int * assignment, bool * starMatrix, int nOfRows, int nOfColumns);
  void computeassignmentcost(int * assignment, double * cost, double * distMatrix, int nOfRows);
  void step2a(
    int * assignment, double * distMatrix, bool * starMatrix, bool * newStarMatrix,
    bool * primeMatrix, bool * coveredColumns, bool * coveredRows, int nOfRows, int nOfColumns,
    int minDim);
  void step2b(
    int * assignment, double * distMatrix, bool * starMatrix, bool * newStarMatrix,
    bool * primeMatrix, bool * coveredColumns, bool * coveredRows, int nOfRows, int nOfColumns,
    int minDim);
  void step3(
    int * assignment, double * distMatrix, bool * starMatrix, bool * newStarMatrix,
    bool * primeMatrix, bool * coveredColumns, bool * coveredRows, int nOfRows, int nOfColumns,
    int minDim);
  void step4(
    int * assignment, double * distMatrix, bool * starMatrix, bool * newStarMatrix,
    bool * primeMatrix, bool * coveredColumns, bool * coveredRows, int nOfRows, int nOfColumns,
    int minDim, int row, int col);
  void step5(
    int * assignment, double * distMatrix, bool * starMatrix, bool * newStarMatrix,
    bool * primeMatrix, bool * coveredColumns, bool * coveredRows, int nOfRows, int nOfColumns,
    int minDim);
};

#endif
