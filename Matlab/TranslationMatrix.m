function [Tranos] = TranslationMatrix(x,y,z)
    Tranos = eye(4,4);
    Tranos(1,4) = x;
    Tranos(2,4) = y;
    Tranos(3,4) = z;
end
