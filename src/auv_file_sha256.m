function value = auv_file_sha256(file)
fid=fopen(file,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));data=fread(fid,Inf,'*uint8');
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(data);
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
