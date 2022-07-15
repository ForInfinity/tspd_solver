import itertools
from typing import Iterator, TypeVar, Callable

T = TypeVar('T')  # Declare type variable
S = TypeVar('S')


class CombinedIterator(Iterator[tuple[T, S]]):
    """
     An iterator that combines 2 iterators.
    """

    def __init__(self, first: Iterator[T], second_generator: Callable[[T], Iterator[S]]):
        self.__first_iter = first
        self.__get_second_iter = second_generator
        self.__first_item = next(self.__first_iter, None)
        if self.__first_item is None:
            raise StopIteration("First iterator is empty.")
        self.__second_iter = self.__get_second_iter(self.__first_item)

    def __get_next_second_item(self):
        try:
            return next(self.__second_iter)
        except StopIteration:
            self.__first_item = next(self.__first_iter)
            self.__second_iter = self.__get_second_iter(self.__first_item)
            return self.__get_next_second_item()

    def __next__(self):
        second_item = self.__get_next_second_item()
        return self.__first_item, second_item


class DroneTuplesIterator(Iterator):

    def __init__(self, start_end_tuples: list[list[tuple[int, int]]], drone_ids: list[int]):
        self.__combined_iter = CombinedIterator(iter(start_end_tuples),
                                                lambda _: iter(itertools.permutations(drone_ids, len(drone_ids))))

    def __next__(self) -> list[tuple[int, int]]:
        start_end_list, drone_ids = next(self.__combined_iter)
        interpreted_list = []

        for idx, drone_id in enumerate(drone_ids):
            start, end = start_end_list[idx]
            interpreted_list.append((start, drone_ids[idx]))
            interpreted_list.append((drone_ids[idx], end))

        return interpreted_list
