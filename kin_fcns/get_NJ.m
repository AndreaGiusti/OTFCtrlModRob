function NJ = get_NJ(ModRob)
% function that given ModRob returns the number of joints of the assembly

NJ = 0;
for k =1:length(ModRob)
    if (ModRob(k).Mod.ID ~= 0 && ModRob(k).Mod.typ == 1)
        NJ = NJ+1;
    end
end

end