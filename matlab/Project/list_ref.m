function [] = list_ref(movelist_all, z_slices, name)
delay =0.00000001;
s = strcat( './' , name , '.txt');

fid = fopen(s, 'wt' );
for i = 1: size(movelist_all,2)
    mlst_all = movelist_all{i};
    if delay > 0
    if ~isempty(mlst_all)
        for j = 1:size(mlst_all,1)-1
            %% plot3(mlst_all(j:j+1,1),mlst_all(j:j+1,2),ones(2,1)*z_slices(i),'b')
            fprintf( fid, '%f,%f,%f\n',mlst_all(j,1),mlst_all(j,2),ones(1,1)*z_slices(i));
        end
    end  
    else
        if ~isempty(mlst_all)
         %% fprintf( fid, '%f,%f,%f\n',mlst_all(:,1),mlst_all(:,2),ones(size(mlst_all,1),1)*z_slices(i));
        end
    end
end   
fclose(fid);
end

