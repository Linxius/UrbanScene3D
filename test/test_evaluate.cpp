#include "evaluate_model.h"
#include <iostream>

int
main(int argc, char *argv[])
{
    if (argc != 3) return 1;
    auto em = EvaluateModel();
    em.read_gt_points(argv[1]);
    em.read_recon_points(argv[2]);
    em.evaluate();
    return 0;
}