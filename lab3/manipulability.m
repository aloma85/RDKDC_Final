function [mu] = manipulability(mode, J)
    Jt = J * J';
    eigJt = eig(Jt);
    sqrtEig = sqrt(eigJt);

    switch mode
        case "sigmamin"
            mu = min(sqrtEig);
        case "detjac"
            mu = det(J);
        case "invcond"
            mu = min(sqrtEig) / max(sqrtEig);
        otherwise
            fprintf("Invalid input\n");
            mu = [];
    end
end
