
#include "JenksBreaks.h"

double inline variance(const std::valarray<double> &data_list)
{
    double mean = data_list.sum() / data_list.size();
    double var = ((data_list * data_list).sum() - mean * mean * data_list.size()) / data_list.size();
    return var;
}

double inline max_variance(const std::valarray<double> &data_list, const std::valarray<double> &kclass, std::vector<std::valarray<double>> &separated_data)
{
    double max_var = 0;

    int begin = 0;
    int level = 0;
    for (int level = 0; level < (int)kclass.size(); level++)
    {
        int base = 0;
        for (int i = base; i < (int)data_list.size(); i++)
        {
            if (kclass[level] <= data_list[i] && data_list[i] <= kclass[level + 1])
            {
                continue;
            }
            else{
                std::valarray<double> part = data_list[std::slice(base, i, 1)];
                max_var = std::max(max_var, variance(part));
                separated_data.push_back(part);
                break;
            }
        }
    }

    return max_var;
}


std::valarray<double> get_jenks_breaks(std::valarray<double> &data_list, int number_class)
{
    std::sort(std::begin(data_list), std::end(data_list));

    std::valarray<std::valarray<double>> mat1(std::valarray<double>(number_class + 1), (int)data_list.size() + 1);
    std::valarray<std::valarray<double>> mat2(std::valarray<double>(number_class + 1), (int)data_list.size() + 1);

    for (int i = 1; i < number_class + 1; i++)
    {
        mat1[1][i] = 1;
        mat2[1][i] = 0;
        for (int j = 2; j < (int)data_list.size() + 1; j++)
            mat2[j][i] = std::numeric_limits<double>::infinity();
    }

    double v = 0.0;
    for (int l = 2; l < (int)data_list.size() + 1; l++)
    {
        double s1 = 0.0;
        double s2 = 0.0;
        double w = 0.0;
        for (int m = 1; m < l + 1; m++)
        {
            int i3 = l - m + 1;
            double val = data_list[i3 - 1];
            s2 += val * val;
            s1 += val;
            w += 1;
            v = s2 - (s1 * s1) / w;
            double i4 = i3 - 1;
            if (i4 != 0)
            {
                for (int j = 2; j < number_class + 1; j++)
                {
                    if (mat2[l][j] >= (v + mat2[i4][j - 1]))
                    {
                        mat1[l][j] = i3;
                        mat2[l][j] = v + mat2[i4][j - 1];
                    }
                }
            }
            mat1[l][1] = 1;
            mat2[l][1] = v;
        }
    }
    int k = data_list.size();
    std::valarray<double> kclass(number_class + 1);

    for (int i = 0; i < number_class + 1; i++)
        kclass[0] = data_list[0];
    kclass[number_class] = data_list[data_list.size() - 1];
    int count_num = number_class;
    while (count_num >= 2)
    {
        int idx = (int)((mat1[k][count_num]) - 2);
        kclass[count_num - 1] = data_list[idx];
        k = int((mat1[k][count_num] - 1));
        count_num -= 1;
    }

    return kclass;
}

std::vector<std::valarray<double>> auto_cluster(std::vector<double> &data_list, double max_var = 0.1)
{
    std::valarray<double> data_copy(data_list.data(), data_list.size());
    //std::cout<<"cont:"<<cont<<"\n";

    int n = (int)data_copy.size();
    std::vector<std::valarray<double>> separated_data;

    
    for (int i = 1; i <= n; i++)
    {
        separated_data.clear();
        auto k_tree = get_jenks_breaks(data_copy, i);
        
        double var = max_variance(data_copy, k_tree, separated_data);
        printf("level: %d, %lf\n", i, var);
        for (auto s : separated_data)
        {
            std::cout << "------------" << std::endl;
            for (auto m : s)
            {
                std::cout << m << std::endl;
            }
        }
        return separated_data;

        if(var<max_var){
            return separated_data;
            
        }
        if(i >= 4){
            return separated_data;
            
        }
    }
    //separated_data.push_back(data_copy);
    separated_data.clear();
    return separated_data;
}
