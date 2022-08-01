pub mod filters;
pub mod kalman;

pub fn ok_or_panic<T, E>(result: Result<T, E>, panic_info: &str) -> T {
    match result {
        Ok(it) => it,
        Err(_err) => panic!("{}", panic_info),
    }
}
