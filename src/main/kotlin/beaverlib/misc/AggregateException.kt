package frc.beaverlib.misc

@Deprecated("AggregateException is deprecated")
class AggregateException(message: String?, primary: Exception?, vararg others: Throwable) : Exception(message, primary) {
    init {
        others.forEach {err -> addSuppressed(err)}
    }
}