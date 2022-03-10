#ifndef Optional_h
#define Optional_h

template <typename T, typename P, typename F, typename C, typename V>
class IOptional
{
public:
    virtual bool isPresent() const;

    virtual bool isEmpty() const;

    virtual IOptional<T, P, F, C, V> filter(P predicate) const;

    virtual void ifPresent(C callback) const;

    virtual void ifAbsent(C callback) const;

    virtual void ifPresentOrElse(C callback, C elseCallback) const;

    virtual T getDefault(T defaultValue) const;

    virtual IOptional<T, P, F, C, V> orElse(T defaultValue) const;

    virtual bool equals(T other) const;

    virtual bool equals(IOptional<T, P, F, C, V> other) const;

    /**
     * @brief Will let you convert from one value to another without checking if the existing value exists
     *
     * @tparam F the return type of the map function
     * @param callback a function that takes the current value and returns a new value. If the current value is empty, the new value will be empty.
     * @return Optional<F> an optional containing an empty value or a value returned from the map function
     */
    virtual IOptional<T, P, F, C, V> map(F callback) const;
};

template <typename T, typename P, typename F, typename C, typename V>
class Optional : public IOptional<T, P, F, C, V>
{
};

template <typename T, typename P, typename F, typename C, typename V>
class EmptyOptional : public IOptional<T, P, F, C, V>
{
public:
    virtual bool isPresent() const
    {
        return false;
    }

    virtual bool isEmpty() const
    {
        return true;
    }

    virtual IOptional<T, P, F, C, V> filter(P predicate) const
    {
        return EmptyOptional<T, P, F, C, V>();
    }

    virtual void ifPresent(C callback) const
    {
        return;
    }

    virtual void ifAbsent(C callback) const
    {
        callback();
    }

    virtual void ifPresentOrElse(C callback, C elseCallback) const
    {
        elseCallback();
    }

    virtual T getDefault(T defaultValue) const
    {
        return defaultValue;
    }

    virtual IOptional<T, P, F, C, V> orElse(T defaultValue) const
    {
        return Optional<T, P, F, C, V>(defaultValue);
    }

    virtual bool equals(T other) const
    {
        return false;
    }

    virtual bool equals(IOptional<T, P, F, C, V> other) const
    {
        return other.isEmpty();
    }

    /**
     * @brief Will let you convert from one value to another without checking if the existing value exists
     *
     * @tparam F the return type of the map function
     * @param callback a function that takes the current value and returns a new value. If the current value is empty, the new value will be empty.
     * @return Optional<F> an optional containing an empty value or a value returned from the map function
     */
    virtual IOptional<T, P, F, C, V> map(F callback) const
    {
        return EmptyOptional<T, P, F, C, V>();
    }
};

template <typename T, typename P, typename F, typename C, typename V>
class Optional : public IOptional<T, P, F, C, V>
{
private:
    T value;
    bool hasValue;
    Optional(T value) : value(value) : hasValue(true)
    {
    }

public:
    static IOptional<T, P, F, C, V>
    of(T value)
    {
        if (value == nullptr)
        {
            return EmptyOptional<T, P, F, C, V>();
        }
        return Optional<T, P, F, C, V>(value);
    }

    static IOptional<T, P, F, C, V> ofNullable(T value)
    {
        return of(value);
    }

    static EmptyOptional<T, P, F, C, V> empty()
    {
        return EmptyOptional<T, P, F, C, V>();
    }

    bool isPresent() const
    {
        return hasValue;
    }

    bool isEmpty() const
    {
        return !hasValue;
    }

    IOptional<T, P, F, C, V> filter(P predicate) const
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

    void ifPresent(C callback) const
    {
        if (hasValue)
        {
            callback(value);
        }
    }

    void ifAbsent(C callback) const
    {
        if (!hasValue)
        {
            callback();
        }
    }

    void ifPresentOrElse(C callback, C elseCallback) const
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

    Optional<T, P, F, C, V> orElse(T defaultValue) const
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

    bool equals(IOptional<T, P, F, C, V> other) const
    {
        return other
            .map(this->equals)
            .orElse(false);
    }

    IOptional<T, P, F, C, V> map(V callback) const
    {
        if (hasValue)
        {
            return of(callback(value));
        }
        return empty();
    }
};

#endif