#include <vector>
#include <array>

#ifndef COMP_FILTER_H
#define COMP_FILTER_H

class CompFilter
{
public:
	CompFilter(int size);
	~CompFilter();

	int operator+(const CompFilter& tmp);

private:
	std::vector<int> x;
	int y;
};

#endif
