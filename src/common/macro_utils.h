/**
 * @file macro_utils.h
 * @author SpaceBaker (emile.forcier@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __MACRO_UTILS_H
#define __MACRO_UTILS_H


#define CONCAT(x,y) x ## y
#define EXPAND(x,y) CONCAT(x, y)
#define RESERVED    EXPAND(reserved, __LINE__)


#endif // __MACRO_UTILS_H