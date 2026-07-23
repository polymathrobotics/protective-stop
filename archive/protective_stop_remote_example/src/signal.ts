export type Signal<T> = Promise<T> & {
  resolve: (arg0: T) => void;
  reject: (arg0: Error) => void;
};

export default function signal<T = void>(): Signal<T> {
  let resolve: ((_: T) => void) | undefined;
  let reject: ((_: Error) => void) | undefined;
  const promise = new Promise<T>((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return Object.assign(promise, { resolve: resolve!, reject: reject! });
}
