// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/command_args.h"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

extern "C" int optimize(char* const inputFilename, char* const outputFilename, int maxIterations)
{
  if(!inputFilename || !outputFilename) {
        return 0;
  }

  // create the linear solver
  auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();

  // create the block solver on top of the linear solver
  auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));

  // create the algorithm to carry out the optimization
  OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

  // NOTE: We skip to fix a variable here, either this is stored in the file
  // itself or Levenberg will handle it.

  // create the optimizer to load the data and carry out the optimization
  SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  optimizer.setAlgorithm(optimizationAlgorithm);

  ifstream ifs(inputFilename);
  if (! ifs) {
    cerr << "unable to open " << inputFilename << endl;
    return 0;
  }
  optimizer.load(ifs);
  optimizer.initializeOptimization();
  optimizer.optimize(maxIterations);

  optimizer.save(outputFilename);

  return 1;
}
