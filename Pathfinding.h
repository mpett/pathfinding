#pragma once
#include "stdafx.h"
#include "stdafx.h"
#include "Pathfinding.h"
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <functional>   // std::greater
#include <cassert> //assertions

int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth,
	const int nMapHeight, int* pOutBuffer, const int nOutBufferSize);