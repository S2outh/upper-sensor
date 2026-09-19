use core::{
    marker::PhantomData,
    ops::{AddAssign, Div},
};

/// A generic software Oversampeling manager for generic values,
pub struct Oversampeling<T, O>
where
    T: TryFrom<<O as Div>::Output> + Into<O>,
    O: AddAssign<O> + Div<O> + TryFrom<usize> + Clone,
{
    _phantom: PhantomData<T>,
    data: O,
    default: O,
    counter: usize,
    limit: usize,
}

impl<T, O> Oversampeling<T, O>
where
    T: TryFrom<<O as Div>::Output> + Into<O>,
    O: AddAssign<O> + Div<O> + TryFrom<usize> + Clone,
{
    pub fn new(limit: usize, default: O) -> Self {
        Self {
            _phantom: PhantomData,
            data: default.clone(),
            default,
            counter: 0,
            limit,
        }
    }
    pub fn insert(&mut self, value: T) -> Option<T> {
        self.data += value.into();
        self.counter = (self.counter + 1) % self.limit;
        if self.counter == 0 {
            let data = core::mem::replace(&mut self.data, self.default.clone());
            let averaged_value = data
                / self
                    .limit
                    .try_into()
                    .unwrap_or_else(|_| panic!("could not convert limit"));
            self.data = self.default.clone();
            Some(
                averaged_value
                    .try_into()
                    .unwrap_or_else(|_| panic!("could not convert O back to T")),
            )
        } else {
            None
        }
    }
}
