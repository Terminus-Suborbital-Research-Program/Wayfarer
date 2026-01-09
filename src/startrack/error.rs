
use thiserror::Error;

#[derive(Error, Debug)]
pub enum StartrackerError {
    #[error("No unique solution found for triangle {0}")]
    TriangleQueryFailure(String),
    #[error("One or more of the legs of the triangle being observed does not have a range in the k_vector table, so does not exist in the catalog")]
    NoStarsInCatalog,
    #[error("No unique solution found for pyramid")]
    PyramidConfirmation,
    #[error("Calculated leg falls outside of fov range / valid bounds")]
    NoPairsRange,
    #[error("No solution found in image")]
    NoSolution
}