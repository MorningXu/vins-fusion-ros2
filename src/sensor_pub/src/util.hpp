/** =================================================
* @author: MorningXu (morningxu1991@163.com)
* @version v1.0.0
* @date: 2024年06月28日
* @brief:
* @copyright:
* ================================================== */

#pragma once
#include <queue>

namespace sputil {
    template<typename T>
    void QueueClear(std::queue<T> &q) {
        std::queue<T> temp;
        std::swap(temp,q);
    }

    template<typename T>
    void VectorClear(std::vector<T> &q) {
        std::vector<T> temp;
        std::swap(temp,q);
    }
}
