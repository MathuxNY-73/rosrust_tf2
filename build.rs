use std::path;
use std::env;
use std::error;
use std::marker;
use std::io;


fn retrieve_rosrust_msg_path() -> Result<String, Box<dyn error::Error + marker::Send + marker::Sync>>
{
    let rosrust_msg_dir = env::var("ROSRUST_MSG_PATH")?;
    let rosrust_msg_path = path::Path::new(&rosrust_msg_dir);
    let rosrust_msg_path_abs = rosrust_msg_path.canonicalize()?;
    let rosrust_msg_path_abs = rosrust_msg_path_abs.to_str().ok_or("Cannot retrieve absolute path")?;
    Ok(String::from(rosrust_msg_path_abs))
}

fn main() 
{
    let rosrust_msg_path = match retrieve_rosrust_msg_path() {
        Ok(rosrust_msg_path) => rosrust_msg_path,
        Err(e) => 
        {
            if let Some(env_err) = e.downcast_ref::<env::VarError>() {
                panic!("Error tyring to retrieve env var : {}", env_err);
            }
            else if let Some(io_err) = e.downcast_ref::<io::Error>() {
                panic!("Error trying to do I/O {}", io_err);
            }
            else {
                panic!("{}", e);
            }
        }
    };
    println!("cargo:rustc-env=ROSRUST_MSG_PAHT={}", rosrust_msg_path);
}
