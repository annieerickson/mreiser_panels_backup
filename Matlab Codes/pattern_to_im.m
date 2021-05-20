% save patterns as images to later animate

pattern_file = 'Pattern_pats_pp_smpl.mat';
frames = 20; % 5 frame timestep
save_folder = 'C:\Users\Alex\Documents\Annie\git\mreiser_panels_backup\Matlab Codes\pattern_images\';

% example: stripe, yl, yr, pr, rg, stripe from make_pats_pp_smpl; 
pattern_array = [1, 6, 7, 1];

% first get stripe, y, pr in 1 direction 
for pattern_i=1 : length(pattern_array)-1
    pattern_id = pattern_array(pattern_i);
    pattern_to_image(pattern_file, pattern_id, frames, save_folder);
end

% run progressive yaw in reverse to show opposite direction stimuli
for pattern_i=2 : length(pattern_array)-1
    pattern_val = pattern_array(pattern_i);
    pattern_to_image_rev(pattern_file, pattern_val, frames, save_folder);
end 

%final CL stripe
pattern_to_image_final(pattern_file, pattern_array(length(pattern_array)), frames, save_folder);


% step through pattern save each frame as image
function pattern_to_image(pattern_mat, pattern_id, T, save_folder)
load(pattern_mat);
for t=1:T
    pat = pattern.Pats(:,:,t,pattern_id); % 32x96, vals 7's and 0's
    save_filename = extractBefore(pattern_mat, length(pattern_mat)-3) + "_pattern_" + string(pattern_id) +"_T" + string(t) + ".png";
    disp(save_filename);
    imwrite(pat, save_folder + save_filename);
end
end 

% step through pattern in reverse order and save image
function pattern_to_image_rev(pattern_mat, pattern_id, T, save_folder)
load(pattern_mat);
for t=T:-1:1
    pat = pattern.Pats(:,:,t,pattern_id); 
    save_filename = "r_" + extractBefore(pattern_mat, length(pattern_mat)-3) + "_pattern_" + string(pattern_id) +"_T" + string(T-t+1) + ".png";
    
    disp(save_filename)
    imwrite(pat, save_folder + save_filename);
end 
end 

% for last stimuli, only change is that adds a z to name to play last
function pattern_to_image_final(pattern_mat, pattern_id, T, save_folder)
load(pattern_mat);
for t=T:-1:1
    pat = pattern.Pats(:,:,t, pattern_id);
    % prepend z to list it last alphabetically
    save_filename = "z_" + extractBefore(pattern_mat, length(pattern_mat)-3) + "pattern_" + string(pattern_id) + "_T" + string(T-t+1) + ".png";
    disp(save_filename);
    imwrite(pat, save_folder + save_filename);
end 
end 
