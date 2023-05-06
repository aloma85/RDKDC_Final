
function [mu] = manipulability(mode, J)
    if mode == "sigmamin"
        mu = min(sqrt(eig(transpose(J)*J)));
    elseif mode == "detjac"
        mu = det(J);
    elseif mode == "invcond"
        mu = min(sqrt(eig(transpose(J)*J)))/max(sqrt(eig(transpose(J)*J)));
    else
        fprintf("invalid input")
    end
end



