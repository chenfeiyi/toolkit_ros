/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-01 19:22:42
 * @Description: content
 */
#ifndef FILE_OPERATION_HPP_
#define FILE_OPERATION_HPP_
#include <string.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dirent.h"
namespace calibrator_pipeline {
namespace common {
/**
 * @description: read file list in given folder
 * @param file_list : store file list
 * @return None
 */
static void GetFilelist(std::string folder_path,
                        std::vector<std::string> *file_list,
                        bool concate = true, bool verbose = false) {
  dirent *ptr;
  DIR *dir;
  file_list->clear();
  dir = opendir(folder_path.c_str());
  std::size_t pos;
  folder_path.find_last_of("/", pos);
  if (pos != (folder_path.size() - 1)) folder_path.push_back('/');

  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
      continue;
    std::stringstream ss;
    if (concate) {
      ss << folder_path << ptr->d_name;
    } else {
      ss << ptr->d_name;
    }
    file_list->emplace_back(ss.str());
    if (verbose) {
      std::cout << ss.str() << std::endl;
    }
    std::sort(file_list->begin(), file_list->end());
  }
  closedir(dir);
}

/**
读取txt文件夹中的数据,一行一行的读取
*/
static void SplitString(std::string str, std::vector<std::string> *fields,
                        char separators, bool verbose = false) {
  std::istringstream sin(str);
  std::string field;
  while (std::getline(sin, field, separators)) {
    fields->push_back(field);
    if (verbose) {
      std::cout << field << "--";
    }
  }
  if (verbose) std::cout << std::endl;
}

static void ReadTxt(std::string file_path, std::vector<std::string> *fields,
                    char separator) {
  std::fstream fin;
  fin.open(file_path.c_str(), std::ios::out);
  if (!fin.is_open()) {
    std::cout << "can't open file " << file_path.c_str() << std::endl;
    return;
  }
  std::string line;
  while (std::getline(fin, line)) {
    //  将line一整句string按照分号切割成多个string并存储到fields中vector中;
    SplitString(line, fields, separator);
  }
}
}  // namespace common
}  // namespace calibrator_pipeline
#endif  // FILE_OPERATION_HPP_
