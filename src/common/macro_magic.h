/**
 * @file macro_magic.h
 * @author SpaceBaker (emile.forcier@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#define CONCAT(x,y) x ## y
#define EXPAND(x,y) CONCAT(x, y)
#define RESERVED    EXPAND(reserved, __LINE__)