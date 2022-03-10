#ifndef Optional_h
#define Optional_h

template <typename T>
class Optional
{
private:
    T value;
    bool hasValue;

public:
    Optional() : hasValue(false)
    {
    }

    Optional(T value) : hasValue(value != nullptr), value(value)
    {
    }

    static Optional<T>
    of(T value)
    {
        return Optional<T>(value);
    }

    static Optional<T> ofNullable(T value)
    {
        return of(value);
    }

    static Optional<T> empty()
    {
        return Optional<T>();
    }

    bool isPresent() const
    {
        return hasValue;
    }

    bool isEmpty() const
    {
        return !hasValue;
    }

    template <typename F>
    Optional<T> filter(F predicate) const
    {
        if (hasValue)
        {
            if (predicate(value))
            {
                return of(value);
            }
            else
            {
                return empty();
            }
        }
        else
        {
            return empty();
        }
    }

    template <typename F>
    void ifPresent(F callback) const
    {
        if (hasValue)
        {
            callback(value);
        }
    }

    template <typename F>
    void ifAbsent(F callback) const
    {
        if (!hasValue)
        {
            callback();
        }
    }

    template <typename F>
    void ifPresentOrElse(F callback, F elseCallback) const
    {
        if (hasValue)
        {
            callback(value);
        }
        else
        {
            elseCallback();
        }
    }

    T get() const
    {
        return value;
    }

    T getDefault(T defaultValue) const
    {
        if (hasValue)
        {
            return value;
        }
        return defaultValue;
    }

    Optional<T> orElse(T defaultValue) const
    {
        return of(getDefault(defaultValue));
    }

    bool equals(T other) const
    {
        if (hasValue)
        {
            return value == other;
        }
        return false;
    }

    bool equals(Optional<T> other) const
    {
        return other
            .map(this->equals)
            .orElse(false);
    }

    /**
     * @brief Will let you convert from one value to another without checking if the existing value exists
     *
     * @tparam F the return type of the map function
     * @param callback a function that takes the current value and returns a new value. If the current value is empty, the new value will be empty.
     * @return Optional<F> an optional containing an empty value or a value returned from the map function
     */
    template <typename F>
    Optional<F> map(F callback) const
    {
        if (hasValue)
        {
            return Optional<F>(callback(value));
        }
        return Optional<F>();
    }
};

#endif