function verify_source_integrity()
root=fileparts(fileparts(mfilename('fullpath')));
manifest=jsondecode(fileread(fullfile(root,'reference_data','source_manifest.json')));
% jsondecode normalizes dots in field names; recover .m from its final suffix.
names=fieldnames(manifest.kernelSha256);
for k=1:numel(names)
 file=[names{k}(1:end-2) '.m'];expected=manifest.kernelSha256.(names{k});
 assert(strcmp(auv_file_sha256(fullfile(root,'src',file)),expected),'AUV:Integrity','Kernel differs: %s',file);
end
assert(strcmp(auv_file_sha256(fullfile(root,'models','auv_v3_closed_loop.slx')),manifest.modelSha256));
fprintf('Frozen v3 kernel and model checksums verified.\n');
end
